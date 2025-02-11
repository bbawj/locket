#include "driver/i2c_types.h"
#include "esp_err.h"

#define MAX_TOUCH 2

typedef struct {
  i2c_master_dev_handle_t handle;
  uint8_t tx_buf[10];
  uint8_t recv_buf[1];
} ft6336;

typedef enum {
  DOWN = 0,
  UP,
  CONTACT,
} touch_type;

typedef enum {
  NONE = 0,
  PAN_RIGHT = 0x14,
  PAN_LEFT = 0x1C,
} gesture;

typedef struct {
  uint8_t x;
  uint8_t y;
  touch_type type;
} touch;

typedef struct {
  touch point[MAX_TOUCH];
  gesture gesture;
  uint8_t n;
  bool new;
} touch_event;

esp_err_t touch_init(i2c_master_bus_handle_t bus_handle, ft6336 *dev,
                     int rst_pin);

void touch_install_intr_handler(uint8_t intr_pin, bool *has_touch);

esp_err_t touch_get_data(ft6336 *dev, touch_event *e, bool read_xy);
