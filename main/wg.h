#ifndef WG_H
#define WG_H

#include "esp_err.h"

void wg_ping();
esp_err_t wifi_init_sta(void);

#endif
