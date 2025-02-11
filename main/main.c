#include "camera.h"
#include "decode_image.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7796.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "freertos/idf_additions.h"
#include "hal/gpio_types.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "portmacro.h"
#include "touch.h"
#include "wg.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define LCD_HOST SPI2_HOST
#define PARALLEL_LINES 16
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 0
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_CS 38
#define EXAMPLE_PIN_NUM_RST 39
#define EXAMPLE_PIN_NUM_DC 40
#define DISP_MOSI 41
#define EXAMPLE_PIN_NUM_PCLK 42
#define EXAMPLE_PIN_NUM_BK_LIGHT 43
#define DISP_MISO 44

#define TOUCH_RST 14
#define TOUCH_INT 21
#define TOUCH_SCL 47
#define TOUCH_SDA 48

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 480
#define EXAMPLE_LCD_V_RES 320
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

static uint16_t *s_lines[2];
static const char *TAG = "app";
static const char *topic = "/jpeg";

static void log_error_if_nonzero(const char *message, int error_code) {
  if (error_code != 0) {
    ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
  }
}

typedef enum {
  Display,
  CameraPreview,
} State;

volatile bool has_touch = false;
static touch_event e = {0};
static State state = Display;

void handle_touch_event(ft6336 *dev, touch_event *e) {
  if (!has_touch)
    return;
  esp_err_t err = touch_get_data(dev, e, true);
  has_touch = false;
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to get touch data");
    return;
  }
  if (e->n == 0) {
    ESP_LOGI(TAG, "touch occurred but no valid points logged");
    return;
  }
  switch (state) {
  case Display: {
    // tap on display screen
    for (int i = 0; i < e->n; ++i) {
      if (e->point[i].type == DOWN) {
        state = CameraPreview;
        return;
      }
    }
  } break;
  case CameraPreview: {
    switch (e->gesture) {
      // tap on camera preview screen
    case NONE: {
      // capture an image
    } break;
    case PAN_RIGHT: {
    } break;
    case PAN_LEFT: {
      // exit
      state = Display;
    } break;
    default:
      break;
    }
  } break;
  }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  ESP_LOGD(TAG,
           "Event dispatched from event loop base=%s, event_id=%" PRIi32 "",
           base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    msg_id = esp_mqtt_client_subscribe(client, topic, 0);
    ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;

  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    esp_mqtt_error_type_t error_type = event->error_handle->error_type;
    if (error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
      log_error_if_nonzero("reported from esp-tls",
                           event->error_handle->esp_tls_last_esp_err);
      log_error_if_nonzero("reported from tls stack",
                           event->error_handle->esp_tls_stack_err);
      log_error_if_nonzero("captured as transport's socket errno",
                           event->error_handle->esp_transport_sock_errno);
      ESP_LOGI(TAG, "Last errno string (%s)",
               strerror(event->error_handle->esp_transport_sock_errno));
    } else if (error_type == MQTT_ERROR_TYPE_SUBSCRIBE_FAILED) {
      switch (event->error_handle->connect_return_code) {
      case MQTT_CONNECTION_REFUSE_PROTOCOL: /*!< *MQTT* connection refused
                                               reason: Wrong protocol */
        ESP_LOGI(TAG, "MQTT wrong protocol");
        break;
      case MQTT_CONNECTION_REFUSE_ID_REJECTED: /*!< *MQTT* connection refused
                                                  reason: ID rejected */
        ESP_LOGI(TAG, "ID rejected");
        break;
      case MQTT_CONNECTION_REFUSE_SERVER_UNAVAILABLE: /*!< *MQTT* connection
                                                         refused reason: Server
            unavailable */
        ESP_LOGI(TAG, "Server unavailable");
        break;
      case MQTT_CONNECTION_REFUSE_BAD_USERNAME: /*!< *MQTT* connection refused
                                                   reason: Wrong user */
        ESP_LOGI(TAG, "Bad username");
        break;
      case MQTT_CONNECTION_REFUSE_NOT_AUTHORIZED: /*!< *MQTT* connection refused
                                                     reason: Wrong username or
                                                     password */
        ESP_LOGI(TAG, "Bad username or password");
        break;
      default:
        ESP_LOGI(TAG, "Unhandled connection return code");
        break;
      }
    }
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}

void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  // ESP_ERROR_CHECK(ret);
  // // wg_ping();
  //
  // ret = wifi_init_sta();
  // if (ret != ESP_OK) {
  //   ESP_LOGE(TAG, "wifi_init_sta: %s", esp_err_to_name(ret));
  //   return;
  // }
  gpio_config_t bk_gpio_config = {.mode = GPIO_MODE_OUTPUT,
                                  .pin_bit_mask = 1ULL
                                                  << EXAMPLE_PIN_NUM_BK_LIGHT};
  // Initialize the GPIO of backlight
  ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

  spi_bus_config_t buscfg = {.sclk_io_num = EXAMPLE_PIN_NUM_PCLK,
                             .mosi_io_num = DISP_MOSI,
                             .miso_io_num = -1,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1,
                             // .flags = SPICOMMON_BUSFLAG_GPIO_PINS,
                             .max_transfer_sz =
                                 PARALLEL_LINES * EXAMPLE_LCD_H_RES * 2 + 8};
  // Initialize the SPI bus
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {
      .dc_gpio_num = EXAMPLE_PIN_NUM_DC,
      .cs_gpio_num = EXAMPLE_PIN_NUM_CS,
      .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
      .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
      .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
      .spi_mode = 0,
      .trans_queue_depth = 10,
  };
  // Attach the LCD to the SPI bus
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                           &io_config, &io_handle));

  esp_lcd_panel_handle_t panel_handle = NULL;
  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
      .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
      .bits_per_pixel = 16,
  };
  // Initialize the LCD configuration
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle));

  // Turn off backlight to avoid unpredictable display on the LCD screen
  // initializing the LCD panel driver. (Different LCD screens may need
  // different levels)
  ESP_ERROR_CHECK(
      gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL));

  // Reset the display
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));

  // Initialize LCD panel
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

  // Turn on the screen
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

  // Swap x and y axis (Different LCD screens may need different options)
  ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));

  // Turn on backlight (Different LCD screens may need different levels)
  ESP_ERROR_CHECK(
      gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL));

  // Allocate memory for the pixel buffers
  for (int i = 0; i < 2; i++) {
    s_lines[i] = heap_caps_malloc(
        EXAMPLE_LCD_H_RES * PARALLEL_LINES * sizeof(uint16_t), MALLOC_CAP_DMA);
    assert(s_lines[i] != NULL);
  }

  gpio_config_t touch_rst = {.mode = GPIO_MODE_OUTPUT,
                             .pin_bit_mask = 1ULL << TOUCH_RST};
  ESP_ERROR_CHECK(gpio_config(&touch_rst));

  gpio_config_t touch_int = {
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = 1ULL << TOUCH_INT,
      .intr_type = GPIO_INTR_NEGEDGE,
  };
  ESP_ERROR_CHECK(gpio_config(&touch_int));

  i2c_master_bus_config_t i2c_mst_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = -1,
      .scl_io_num = TOUCH_SCL,
      .sda_io_num = TOUCH_SDA,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
  ft6336 touch_handle;
  touch_init(bus_handle, &touch_handle, TOUCH_RST);
  touch_install_intr_handler(TOUCH_INT, &has_touch);
  // uint8_t *camera_image = malloc(480 * 320 * 2);
  // if (camera_image == NULL) {
  //   ESP_LOGE("app", "failed to allocate for camera image");
  //   return;
  // }
  // ESP_ERROR_CHECK(camera_init());

  // esp_mqtt_client_config_t mqtt_cfg = {
  //     .broker.address.uri = CONFIG_BROKER_URL,
  // };
  // esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
  /* The last argument may be used to pass data to the event handler, in this
   * example mqtt_event_handler */
  // esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID,
  // mqtt_event_handler,
  //                                NULL);
  // esp_mqtt_client_start(client);
  // if (camera_capture(client, camera_image) != ESP_OK) {
  //   ESP_LOGE(TAG, "Camera failed to capture");
  // }
  uint16_t *test_image;
  ESP_ERROR_CHECK(decode_image(&test_image));
  // esp_mqtt_client_publish(client, topic, (const char *)test_image,
  //                         sizeof(uint16_t) * IMAGE_W * IMAGE_H, 0, 0);
  ESP_LOGI(TAG, "pixel data is 0x%X", test_image[1]);
  while (1) {
    // if (camera_capture(camera_image) != ESP_OK) {
    //   break;
    // }

    gpio_intr_disable(TOUCH_INT);
    handle_touch_event(&touch_handle, &e);
    switch (state) {
    case Display: {
      for (int y = 0; y < EXAMPLE_LCD_V_RES; y += PARALLEL_LINES) {
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, 0 + EXAMPLE_LCD_H_RES,
                                  y + PARALLEL_LINES,
                                  &test_image[y * EXAMPLE_LCD_H_RES]);
      }
    } break;
    case CameraPreview:
      break;
    }
    gpio_intr_enable(TOUCH_INT);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  // free(camera_image);
}
