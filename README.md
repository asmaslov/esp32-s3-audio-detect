# ESP32-S3 Audio Detect
Based on ESP32-S3 development board with MSM261S4030H0R.

## Overview
Used [ESP32-S3-EYE v2.2](https://github.com/espressif/esp-who/blob/master/docs/en/get-started/ESP32-S3-EYE_Getting_Started_Guide.md) development kit without LCD module.

Audio samples are collected with MSM261S4030H0R using I2S interface.

## Prediction
Values are collected in real time and prediction result are printed in the debug console.

### Example output
```
I (224) audio-detect: Program start

Performance calibration is configured for your project. If no event is detected, all values are 0.

Predictions:
  _unknown: 0.00000
  off: 0.00000
  on: 0.00000
```

## Pin Assignment
**Note:** The following pin assignments are used by default, you can change these in the `menuconfig`.

| I2S_NUM          | DATA          | WS          | CLK          |
| ---------------- | ------------- | ----------- | ------------ |
| I2S_NUM_0        | I2S_DATA_GPIO | I2S_WS_GPIO | I2S_CLK_GPIO |

For the actual default values see `Project Configuration` in `menuconfig`.

## Build and Flash
Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

# License
Unless specifically indicated otherwise in a file or a folder, files are licensed under the Apache License, Version 2.0 (check out LICENSE file).
