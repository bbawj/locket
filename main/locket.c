#include "camera.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7796.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "nvs_flash.h"
#include "wg.h"
#include <stdio.h>

#define LCD_HOST SPI2_HOST
#define PARALLEL_LINES 16
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 0
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_DATA0                                                  \
  37 /*!< for 1-line SPI, this also refereed as MOSI */
#define EXAMPLE_PIN_NUM_PCLK 36
#define EXAMPLE_PIN_NUM_CS 9
// IMPORTANT: THIS DC ONLY SEEMS TO WORK WITH SOME PINS ONLY
#define EXAMPLE_PIN_NUM_DC 38

#define EXAMPLE_PIN_NUM_RST 14
#define EXAMPLE_PIN_NUM_BK_LIGHT 35

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 320
#define EXAMPLE_LCD_V_RES 480
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

static uint16_t *s_lines[2];

void app_main(void) {
  // Initialize NVS
  // esp_err_t ret = nvs_flash_init();
  // if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
  //     ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
  //   ESP_ERROR_CHECK(nvs_flash_erase());
  //   ret = nvs_flash_init();
  // }
  // ESP_ERROR_CHECK(ret);
  // // wg_ping();
  //
  // gpio_config_t bk_gpio_config = {.mode = GPIO_MODE_OUTPUT,
  //                                 .pin_bit_mask = 1ULL
  //                                                 <<
  //                                                 EXAMPLE_PIN_NUM_BK_LIGHT};
  // // Initialize the GPIO of backlight
  // ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
  //
  // spi_bus_config_t buscfg = {.sclk_io_num = EXAMPLE_PIN_NUM_PCLK,
  //                            .mosi_io_num = EXAMPLE_PIN_NUM_DATA0,
  //                            .miso_io_num = -1,
  //                            .quadwp_io_num = -1,
  //                            .quadhd_io_num = -1,
  //                            // .flags = SPICOMMON_BUSFLAG_GPIO_PINS,
  //                            .max_transfer_sz =
  //                                PARALLEL_LINES * EXAMPLE_LCD_H_RES * 2 + 8};
  // // Initialize the SPI bus
  // ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
  //
  // esp_lcd_panel_io_handle_t io_handle = NULL;
  // esp_lcd_panel_io_spi_config_t io_config = {
  //     .dc_gpio_num = EXAMPLE_PIN_NUM_DC,
  //     .cs_gpio_num = EXAMPLE_PIN_NUM_CS,
  //     .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
  //     .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
  //     .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
  //     .spi_mode = 0,
  //     .trans_queue_depth = 10,
  // };
  // // Attach the LCD to the SPI bus
  // ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
  //                                          &io_config, &io_handle));
  //
  // esp_lcd_panel_handle_t panel_handle = NULL;
  // esp_lcd_panel_dev_config_t panel_config = {
  //     .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
  //     .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
  //     .bits_per_pixel = 16,
  // };
  // // Initialize the LCD configuration
  // ESP_ERROR_CHECK(
  //     esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle));
  //
  // // Turn off backlight to avoid unpredictable display on the LCD screen
  // while
  // // initializing the LCD panel driver. (Different LCD screens may need
  // // different levels)
  // ESP_ERROR_CHECK(
  //     gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT,
  //     EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL));
  //
  // // Reset the display
  // ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  //
  // // Initialize LCD panel
  // ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  //
  // // Turn on the screen
  // ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
  // ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
  //
  // // Swap x and y axis (Different LCD screens may need different options)
  // ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
  //
  // // Turn on backlight (Different LCD screens may need different levels)
  // ESP_ERROR_CHECK(
  //     gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT,
  //     !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL));
  //
  // // Allocate memory for the pixel buffers
  // for (int i = 0; i < 2; i++) {
  //   s_lines[i] = heap_caps_malloc(
  //       EXAMPLE_LCD_H_RES * PARALLEL_LINES * sizeof(uint16_t),
  //       MALLOC_CAP_DMA);
  //   assert(s_lines[i] != NULL);
  // }

  uint8_t *camera_image = NULL;
  ESP_LOGI("app", "debug!");
  ESP_ERROR_CHECK(camera_init());
  while (1) {
    if (camera_capture(camera_image) != ESP_OK) {
      break;
    }
    // for (int y = 0; y < EXAMPLE_LCD_V_RES; y += PARALLEL_LINES) {
    //   esp_lcd_panel_draw_bitmap(panel_handle, 0, y, 0 + EXAMPLE_LCD_H_RES,
    //                             y + PARALLEL_LINES,
    //                             &camera_image[y * EXAMPLE_LCD_H_RES]);
    // }
  }
}
