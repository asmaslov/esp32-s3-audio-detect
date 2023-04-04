#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

const char *tag = "audio-detect";

#define MODEL_FAUCET_NOISE_DETECTION  0
#define MODEL_KEYWORD_SPOTTING        1

//TODO: Make option to choose model
#define MODEL MODEL_KEYWORD_SPOTTING

#if (MODEL == MODEL_FAUCET_NOISE_DETECTION)
    #define PREDICT_ON   0
    #define PREDICT_OFF  1
#elif (MODEL == MODEL_KEYWORD_SPOTTING)
    #define PREDICT_ON   2
    #define PREDICT_OFF  1
#endif

#if defined(CONFIG_I2S_NUM_0)
#define I2S_NUM  I2S_NUM_0
#elif defined(CONFIG_I2S_NUM_1)
#define I2S_NUM  I2S_NUM_1
#endif
#define I2S_SAMPLE_BITS  24
#define I2S_SAMPLE_MAX   ((1 << I2S_SAMPLE_BITS) - 1)
#define I2S_SAMPLE_MULT  14.63
int32_t i2s_buff[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

#define SD_MOUNT_POINT   "/sdcard"
#define WAV_HEADER_SIZE  44
typedef int16_t wav_sample_t;
#define WAV_SAMPLE_MAX   SHRT_MAX
#define WAV_BYTE_RATE    (EI_CLASSIFIER_FREQUENCY * sizeof(wav_sample_t))
sdmmc_card_t* card;
FILE* wav_file;
QueueHandle_t wav_queue;
#define WAV_CHUNK_SAMPLES  200
int wav_recorded_bytes;
//TODO: Make controllable playback
#define WAV_REC_TIME  10
SemaphoreHandle_t semaphore_start, semaphore_stop, semaphore_write;

#define TASK_READ_BUTTONS_STACK_SIZE        2048
#define TASK_READ_BUTTONS_PRIORITY          (configMAX_PRIORITIES - 2)
#define TASK_WRITE_FILE_STACK_SIZE          4096
#define TASK_WRITE_FILE_PRIORITY            (configMAX_PRIORITIES - 2)
#define TASK_DETECT_STACK_SIZE              4096
#define TASK_DETECT_PRIORITY                (configMAX_PRIORITIES - 1)

/* I2S */

esp_err_t init_microphone(void) {
    esp_err_t ret;

    const i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = EI_CLASSIFIER_FREQUENCY,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,  // Workaround due to real 24 bits data format is unusable (IDF v4.4.4)
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 200,
        .use_apll = true,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
        .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
        .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT
    };

    const i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = CONFIG_I2S_CLK_GPIO,
        .ws_io_num = CONFIG_I2S_WS_GPIO,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = CONFIG_I2S_DATA_GPIO
    };

    ret = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    ESP_ERROR_CHECK(ret);
    ret = i2s_set_pin(I2S_NUM, &pin_config);
    ESP_ERROR_CHECK(ret);
    ret = i2s_set_clk(I2S_NUM, EI_CLASSIFIER_FREQUENCY, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
    ESP_ERROR_CHECK(ret);

    return ret;
}

/* LED */

esp_err_t led_init(void) {
    esp_err_t ret;

    ret = gpio_reset_pin((gpio_num_t)CONFIG_LED_GPIO);
    ESP_ERROR_CHECK(ret);
    ret = gpio_set_direction((gpio_num_t)CONFIG_LED_GPIO, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(ret);

    return ret;
}

esp_err_t led_turn(bool on) {
    esp_err_t ret;

    ret = gpio_set_level((gpio_num_t)CONFIG_LED_GPIO, on);

    return ret;
}

/* SDCARD */

void mount_sdcard(void)
{
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    ESP_LOGI(tag, "Initializing SD card");

    ESP_LOGI(tag, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = (gpio_num_t)CONFIG_SDIO_CLK_GPIO;
    slot_config.cmd = (gpio_num_t)CONFIG_SDIO_CMD_GPIO;
    slot_config.d0 = (gpio_num_t)CONFIG_SDIO_D0_GPIO;

    ESP_LOGI(tag, "Mounting filesystem");
    ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(tag, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(tag, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        while(1);
    }
    ESP_LOGI(tag, "Filesystem mounted");

    sdmmc_card_print_info(stdout, card);
}

void umount_sdcard(void) {
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, card);
    ESP_LOGI(tag, "Card unmounted");
}

void generate_wav_header(char* wav_header, uint32_t wav_size, uint32_t sample_rate){

    // See this for reference: http://soundfile.sapp.org/doc/WaveFormat/
    uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
    uint32_t byte_rate = WAV_BYTE_RATE;

    const char set_wav_header[] = {
        'R','I','F','F', // ChunkID
        (uint8_t)file_size, (uint8_t)(file_size >> 8), (uint8_t)(file_size >> 16), (uint8_t)(file_size >> 24), // ChunkSize
        'W','A','V','E', // Format
        'f','m','t',' ', // Subchunk1ID
        0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
        0x01, 0x00, // AudioFormat (1 for PCM)
        0x01, 0x00, // NumChannels (1 channel)
        (uint8_t)sample_rate, (uint8_t)(sample_rate >> 8), (uint8_t)(sample_rate >> 16), (uint8_t)(sample_rate >> 24), // SampleRate
        (uint8_t)byte_rate, (uint8_t)(byte_rate >> 8), (uint8_t)(byte_rate >> 16), (uint8_t)(byte_rate >> 24), // ByteRate
        0x02, 0x00, // BlockAlign
        sizeof(wav_sample_t) * 8, 0x00, // BitsPerSample
        'd','a','t','a', // Subchunk2ID
        (uint8_t)wav_size, (uint8_t)(wav_size >> 8), (uint8_t)(wav_size >> 16), (uint8_t)(wav_size >> 24), // Subchunk2Size
    };

    memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
}

/* MAIN */

static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    esp_err_t ret;
    float sample;
    wav_sample_t wav_sample;
    size_t bytes_read;

    ESP_LOGD(tag, "Request data offset=%u length=%u", offset, length);

    ret = i2s_read(I2S_NUM, i2s_buff, length * sizeof(int32_t), &bytes_read, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
    assert(length * sizeof(int32_t) == bytes_read);

    for (size_t i = 0; i < length; i++) {
        sample = (float)(i2s_buff[i] >> 8) / I2S_SAMPLE_MAX * I2S_SAMPLE_MULT;
        sample = sample > 1.0 ? 1.0 : sample;
        sample = sample < -1.0 ? -1.0 : sample;
        out_ptr[i] = sample;
        wav_sample = (int16_t)(sample * WAV_SAMPLE_MAX);
        if (xQueueSend(wav_queue, &wav_sample, 0) != pdTRUE) {
            ESP_LOGE(tag, "Wav file queue overflow");
        }
    }

    return EIDSP_OK;
}

void task_write_file(void *arg) {
    wav_sample_t file_buffer[WAV_CHUNK_SAMPLES];
    size_t sample_idx = 0;

    while (1) {
        xSemaphoreTake(semaphore_write, portMAX_DELAY);
        do {
            xQueueReceive(wav_queue, &file_buffer[sample_idx++], portMAX_DELAY);
            if (WAV_CHUNK_SAMPLES == sample_idx) {
                sample_idx = 0;
                fwrite(file_buffer, 1, WAV_CHUNK_SAMPLES * sizeof(wav_sample_t), wav_file);
                wav_recorded_bytes += WAV_CHUNK_SAMPLES * sizeof(wav_sample_t);
            }
        } while (wav_recorded_bytes < WAV_BYTE_RATE * WAV_REC_TIME);
        xSemaphoreGive(semaphore_stop);
    }
}

void task_read_butons(void *arg) {
    const char *tag = pcTaskGetName(xTaskGetCurrentTaskHandle());

    ESP_LOGI(tag, "Wait 3 seconds to run ...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(tag, "2 ...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(tag, "1 ...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    xSemaphoreGive(semaphore_start);

    while (1) {
        //TODO: Get pressed button
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void task_detect(void *arg) {
    signal_t signal;
    char wav_header_fmt[WAV_HEADER_SIZE];
    struct stat st;
    ei_impulse_result_t result;
    EI_IMPULSE_ERROR ret;
    const char *tag = pcTaskGetName(xTaskGetCurrentTaskHandle());

    // Assign callback function to fill buffer used for preprocessing/inference
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &get_signal_data;

    run_classifier_init();

    while (1) {
        ESP_LOGI(tag, "Insert MicroSD card and press 'Play' button");
        xSemaphoreTake(semaphore_start, portMAX_DELAY);
        ESP_LOGI(tag, "Audio detection started. Press 'Menu' to stop");

        mount_sdcard();

        generate_wav_header(wav_header_fmt, WAV_BYTE_RATE * WAV_REC_TIME, EI_CLASSIFIER_FREQUENCY);

        // Use POSIX and C standard library functions to work with files.
        ESP_LOGI(tag, "Opening file");

        // First check if file exists before creating a new file.
        ESP_LOGI(tag, "Check if file exists");
        if (stat(SD_MOUNT_POINT"/record.wav", &st) == 0) {
            // Delete it if it exists
            ESP_LOGI(tag, "Delete file");
            unlink(SD_MOUNT_POINT"/record.wav");
        }

        // Create new WAV file
        ESP_LOGI(tag, "Create new file");
        wav_file = fopen(SD_MOUNT_POINT"/record.wav", "w");
        if (wav_file == NULL) {
            ESP_LOGE(tag, "Failed to open file for writing");
            while(1);
        }

        // Write the header to the WAV file
        ESP_LOGI(tag, "Write header");
        fwrite(wav_header_fmt, 1, WAV_HEADER_SIZE, wav_file);

        ESP_LOGI(tag, "Run classifier");
        xSemaphoreGive(semaphore_write);
        while (xSemaphoreTake(semaphore_stop, 0) == pdFALSE) {
            // Perform DSP pre-processing and inference
#if (MODEL == MODEL_FAUCET_NOISE_DETECTION)
            ret = run_classifier(&signal, &result, false);
#elif (MODEL == MODEL_KEYWORD_SPOTTING)
            ret = run_classifier_continuous(&signal, &result, false);
#endif
            // Print return code and how long it took to perform inference
            if (ret != EI_IMPULSE_OK) {
                ESP_LOGE(tag, "Classifier returned %d", ret);
            }
            ESP_LOGD(tag, "Timing: DSP %d ms, inference %d ms", 
                    result.timing.dsp, result.timing.classification);

            size_t max_idx;
            float max = 0;
            printf("Predictions:\r\n");
            for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                printf("  %s: ", ei_classifier_inferencing_categories[i]);
                printf("%.5f\r\n", result.classification[i].value);
                if (result.classification[i].value > max) {
                    max = result.classification[i].value;
                    max_idx = i;
                }
            }
            if (PREDICT_ON == max_idx) {
                led_turn(true);
            } else if (PREDICT_OFF == max_idx) {
                led_turn(false);
            }
        }

        ESP_LOGI(tag, "Recording done!");
        fclose(wav_file);
        ESP_LOGI(tag, "File written on SDCard");

        umount_sdcard();
    }
}

extern "C" void app_main(void) {
    TaskHandle_t handle_read_butons, handle_detect, handle_write_file;

    wav_queue = xQueueCreate(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(wav_sample_t));
    semaphore_start = xSemaphoreCreateBinary();
    semaphore_stop = xSemaphoreCreateBinary();
    semaphore_write = xSemaphoreCreateBinary();
    xTaskCreate(task_read_butons, "task_read_butons", TASK_READ_BUTTONS_STACK_SIZE, NULL, TASK_READ_BUTTONS_PRIORITY, &handle_read_butons);

    ESP_ERROR_CHECK(init_microphone());
    ESP_ERROR_CHECK(led_init());

    xTaskCreate(task_write_file, "task_write_file", TASK_WRITE_FILE_STACK_SIZE, NULL, TASK_WRITE_FILE_PRIORITY, &handle_write_file);

    xTaskCreate(task_detect, "task_detect", TASK_DETECT_STACK_SIZE, NULL, TASK_DETECT_PRIORITY, &handle_detect);

    ESP_LOGI(tag, "Program start");

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGD(tag, "Task %s stack watermark = %u", pcTaskGetName(handle_read_butons), uxTaskGetStackHighWaterMark(handle_read_butons));
        ESP_LOGD(tag, "Task %s stack watermark = %u", pcTaskGetName(handle_write_file), uxTaskGetStackHighWaterMark(handle_write_file));
        ESP_LOGD(tag, "Task %s stack watermark = %u", pcTaskGetName(handle_detect), uxTaskGetStackHighWaterMark(handle_detect));
        ESP_LOGD(tag, "Task %s stack watermark = %u", pcTaskGetName(xTaskGetCurrentTaskHandle()), uxTaskGetStackHighWaterMark(NULL));
    }
}
