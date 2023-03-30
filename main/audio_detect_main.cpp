/* I2S Digital Microphone Recording Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

const char *tag = "audio-detect";

#if defined(CONFIG_I2S_NUM_0)
#define I2S_NUM  I2S_NUM_0
#elif defined(CONFIG_I2S_NUM_1)
#define I2S_NUM  I2S_NUM_1
#endif

#define LED_OFF_PERIOD_MS          3000
#define BLINK_LIGHT_LOAD_PERIOD_MS   1000
#define BLINK_MEDIUM_LOAD_PERIOD_MS  500
#define BLINK_HEAVY_LOAD_PERIOD_MS   250

//TODO: Make option to choose model
#define PREDICT_ON   0 // 2
#define PREDICT_OFF  1

#define SAMPLE_MAX  ((1 << 24) - 1)

int32_t i2s_buff[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

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
        .dma_buf_count = 2,
        .dma_buf_len = 200,
        .use_apll = false,
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

/* MAIN */

static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    esp_err_t ret;
    size_t bytes_read;

    ret = i2s_read(I2S_NUM, i2s_buff, length * sizeof(int32_t), &bytes_read, portMAX_DELAY);
    ESP_ERROR_CHECK(ret);
    assert(length * sizeof(int32_t) == bytes_read);

    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (float)(i2s_buff[i] >> 8) / SAMPLE_MAX;
    }

    return EIDSP_OK;
}

extern "C" void app_main(void) {
    signal_t signal;
    ei_impulse_result_t result;
    EI_IMPULSE_ERROR ret;

    ESP_ERROR_CHECK(init_microphone());
    ESP_ERROR_CHECK(led_init());

    ESP_LOGI(tag, "Program start");

    // Assign callback function to fill buffer used for preprocessing/inference
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &get_signal_data;

    run_classifier_init();

    while (1) {
        // Perform DSP pre-processing and inference
        ret = run_classifier(&signal, &result, false);
        // ret = run_classifier_continuous(&signal, &result, false);

        // Print return code and how long it took to perform inference
        if (ret != EI_IMPULSE_OK) {
            ESP_LOGE(tag,  "Classifier returned %d", ret);
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
}
