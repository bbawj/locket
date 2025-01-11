#ifndef CAMERA_H_
#define CAMERA_H_

#include "esp_err.h"

esp_err_t camera_capture(uint8_t *out);
esp_err_t camera_init();

#endif
