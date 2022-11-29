#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_wnm.h"
#include "esp_rrm.h"
#include "esp_mbo.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
//#include "gps.h"

static const char *TAG = "main";

void initialise_wifi();
void gps_init();

uint8_t powerstate()
{
    return 1;
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    tinyusb_config_t tusb_cfg = { 0 }; // the configuration uses default values
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t amc_cfg = { 0 }; // the configuration uses default values
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));

    esp_tusb_init_console(TINYUSB_CDC_ACM_0); // log to usb

    usleep(1000000);

    initialise_wifi();

    //usleep(10000000);

    gps_init();
    
    while(1)
    {
        //ESP_LOGI(TAG, "...");
        usleep(1000000);
    }
}
