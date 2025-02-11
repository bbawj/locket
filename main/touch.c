#include "touch.h"
#include "assert.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include <stdint.h>

const uint8_t DEVICE_MODE = 0;
const uint8_t GESTURE_ID = 0x1;
const uint8_t TD_STATUS = 0x2;
const uint8_t TOUCH_START = 0x3;
const uint8_t TOUCH_END = 0x39;
const uint8_t OFFSET_LEFT_RIGHT = 0x92;
const uint8_t DISTANCE_LEFT_RIGHT = 0x94;
const uint8_t INTR_MODE = 0xA4;
const uint8_t P_MODE = 0xA5;
const uint8_t ID = 0xA8;
const uint8_t GEST_EN = 0xD0;
const uint8_t GEST_NSEW = 0xD1;

static const char *TAG = "ft6336";

esp_err_t touch_tx(ft6336 *dev, uint8_t addr, uint8_t val) {
  dev->tx_buf[0] = addr;
  dev->tx_buf[1] = val;
  esp_err_t err = i2c_master_transmit(dev->handle, dev->tx_buf, 2, -1);
  if (err != ESP_OK) {
    return err;
  }
  ESP_LOGD(TAG, "tx 0x%X to address: 0x%X", val, addr);
  return err;
}

esp_err_t touch_rx(ft6336 *dev, uint8_t addr) {
  dev->tx_buf[0] = addr;
  esp_err_t err = i2c_master_transmit_receive(
      dev->handle, dev->tx_buf, 1, dev->recv_buf, sizeof(dev->recv_buf), -1);
  if (err != ESP_OK) {
    return err;
  }
  ESP_LOGD(TAG, "read addr 0x%X returned: 0x%X", addr, dev->recv_buf[0]);
  return err;
}

// gesture task to read in the background
// must be polled and cannot be used with touch interrupt mode
esp_err_t touch_gesture_mode(ft6336 *dev, gesture *out) {
  esp_err_t err;
  if ((err = touch_tx(dev, GEST_EN, 0x1)) != ESP_OK) {
    ESP_LOGE(TAG, "failed to set gesture en");
    return err;
  }
  touch_rx(dev, GEST_NSEW);
  if ((err = touch_tx(dev, GEST_NSEW, dev->recv_buf[0] | 0xF)) != ESP_OK) {
    ESP_LOGE(TAG, "failed to set gesture nsew en");
    return err;
  }
  if ((err = touch_tx(dev, DISTANCE_LEFT_RIGHT, 50)) != ESP_OK) {
    ESP_LOGE(TAG, "failed to set left right min distance");
    return err;
  }
  if ((err = touch_tx(dev, OFFSET_LEFT_RIGHT, 0xFF)) != ESP_OK) {
    ESP_LOGE(TAG, "failed to set left right max distance");
    return err;
  }
  while (1) {
    touch_rx(dev, 0xD3);
    ESP_LOGI(TAG, "gesture is 0x%X", dev->recv_buf[0]);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  return err;
}

esp_err_t touch_n_pts(ft6336 *dev, uint8_t *n) {
  esp_err_t err = touch_rx(dev, TD_STATUS);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to read TD_STATUS: %d", err);
    return err;
  }
  uint8_t n_pts = dev->recv_buf[0] & 0xF;
  assert(n_pts <= MAX_TOUCH && "Unexpected number of touch points");
  *n = n_pts;
  return err;
}

esp_err_t touch_get_data(ft6336 *dev, touch_event *e, bool read_xy) {
  esp_err_t err = touch_n_pts(dev, &e->n);
  ESP_LOGI(TAG, "N points: %X", e->n);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to get number of touch points: %d", err);
    return err;
  }
  // touch_gesture(dev, &e->gesture);
  // ESP_LOGI(TAG, "gesture: %X", e->gesture);

  if (!read_xy) {
    return err;
  }

  // read coordinates
  for (uint8_t i = 0; i < e->n; ++i) {
    uint8_t x_off = TOUCH_START + i * 5;
    uint8_t y_off = TOUCH_START + 2 + i * 5;
    uint8_t weight = TOUCH_START + 4 + i * 5;
    if ((err = touch_rx(dev, x_off)) != ESP_OK) {
      ESP_LOGE(TAG, "failed to read touch high: %d", err);
      return err;
    }
    e->point[i].type = (touch_type)(dev->recv_buf[0] >> 6);
    uint16_t x_h = dev->recv_buf[0] & 0xF;
    if ((err = touch_rx(dev, x_off + 1)) != ESP_OK) {
      ESP_LOGE(TAG, "failed to read touch low: %d", err);
      return err;
    }
    e->point[i].x = (x_h << 8) | dev->recv_buf[0];

    if ((err = touch_rx(dev, y_off)) != ESP_OK) {
      ESP_LOGE(TAG, "failed to read touch high: %d", err);
      return err;
    }
    uint8_t touch_id = (touch_type)(dev->recv_buf[0] >> 6);
    uint16_t y_h = dev->recv_buf[0] & 0xF;
    if ((err = touch_rx(dev, y_off + 1)) != ESP_OK) {
      ESP_LOGE(TAG, "failed to read touch low: %d", err);
      return err;
    }
    e->point[i].y = (y_h << 8) | dev->recv_buf[0];
  }
  return err;
}

esp_err_t touch_init(i2c_master_bus_handle_t bus_handle, ft6336 *dev,
                     int rst_pin) {
  gpio_set_level(rst_pin, 0);
  vTaskDelay(5 / portTICK_PERIOD_MS);
  gpio_set_level(rst_pin, 1);

  vTaskDelay(500 / portTICK_PERIOD_MS);

  i2c_device_config_t touch_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = 0x38,
      // 400khz max
      .scl_speed_hz = 100000,
  };
  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(bus_handle, &touch_cfg, &dev->handle));

  esp_err_t err = ESP_OK;
  if ((err = touch_rx(dev, DEVICE_MODE)) != ESP_OK) {
    ESP_LOGE(TAG, "failed to read working mode: %d", err);
    return err;
  }
  if (dev->recv_buf[0] != 0x0) {
    ESP_LOGE(TAG, "device not in working mode: %d", dev->recv_buf[0]);
    return ESP_FAIL;
  }

  if ((err = touch_rx(dev, ID)) != ESP_OK) {
    ESP_LOGE(TAG, "failed to read panel id");
    return err;
  }
  if (dev->recv_buf[0] != 0x11) {
    ESP_LOGE(TAG, "device id not expected: %d", dev->recv_buf[0]);
    return ESP_FAIL;
  }

  if ((err = touch_tx(dev, INTR_MODE, 0x1)) != ESP_OK) {
    ESP_LOGE(TAG, "failed to set interrupt en");
    return err;
  }
  // if ((err = touch_tx(dev, P_MODE, 0x0)) != ESP_OK) {
  //   ESP_LOGE(TAG, "failed to set power active mode");
  //   return err;
  // }
  return err;
}

static void touch_intr_handler(void *args) {
  bool *(has_touch) = (bool *)args;
  *has_touch = true;
}

void touch_install_intr_handler(uint8_t intr_pin, bool *has_touch) {
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_EDGE));
  gpio_isr_handler_add(intr_pin, touch_intr_handler, (void *)has_touch);
}
