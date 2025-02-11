#ifndef CAMERA_H_
#define CAMERA_H_

#include "esp_err.h"
#include "mqtt_client.h"

esp_err_t camera_capture(esp_mqtt_client_handle_t client, uint8_t *out);
esp_err_t camera_init();

#endif
