#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include <stdint.h>

// WROVER-KIT PIN Map
#define CAM_PIN_PWDN -1  // power down is not used
#define CAM_PIN_RESET -1 // software reset will be performed
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5

#define CAM_PIN_D7 16
#define CAM_PIN_D6 17
#define CAM_PIN_D5 18
#define CAM_PIN_D4 12
#define CAM_PIN_D3 10
#define CAM_PIN_D2 8
#define CAM_PIN_D1 9
#define CAM_PIN_D0 11
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13

static const char *TAG = "camera";

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000, // EXPERIMENTAL: Set to 16MHz on ESP32-S2 or
                              // ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size =
        FRAMESIZE_HVGA, // QQVGA-UXGA, For ESP32, do not use sizes above QVGA
                        // when not JPEG. The performance of the ESP32-S series
                        // has improved a lot, but JPEG mode always gives better
                        // frame rates.

    .jpeg_quality = 12, // 0-63, for OV series camera sensors, lower number
                        // means higher quality
    .fb_count = 1, // When jpeg mode is used, if fb_count more than one, the
                   // driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY // CAMERA_GRAB_LATEST. Sets when buffers
                                        // should be filled
};

esp_err_t camera_init() {
  // initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera Init Failed");
    return err;
  }

  return ESP_OK;
}

esp_err_t camera_capture(esp_mqtt_client_handle_t client, uint8_t *out) {
  esp_err_t res = ESP_OK;
  camera_fb_t *fb = NULL;

  fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGE(TAG, "Camera capture failed");
    res = ESP_FAIL;
    goto cleanup;
  }

  size_t jpg_buf_len = fb->len;
  uint8_t *jpg_buf = fb->buf;
  int ret = esp_mqtt_client_publish(client, "/jpeg", (char *)jpg_buf,
                                    jpg_buf_len, 0, 0);
  if (ret == -1) {
    ESP_LOGE(TAG, "Failed to publish");
  } else if (ret == -2) {
    ESP_LOGE(TAG, "Failed to publish: outbox full");
  }
  if (!jpg2rgb565(jpg_buf, jpg_buf_len, out, JPG_SCALE_NONE)) {
    ESP_LOGE(TAG, "JPG to RGB failed");
    res = ESP_FAIL;
  }

cleanup:
  esp_camera_fb_return(fb);

  return res;
}
