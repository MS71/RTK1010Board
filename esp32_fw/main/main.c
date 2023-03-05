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

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#include "ota_server.h"

static const char *TAG = "main";


extern EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

void initialise_wifi();
void gps_init();

#if defined(CONFIG_PARTITION_TABLE_CUSTOM) || defined(CONFIG_PARTITION_TABLE_TWO_OTA)
/**
 * @brief ota_server_task
 * @param param
 */
void ota_server_task(void* param)
{
    ESP_LOGI(TAG, "ota task ...");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "starting ota ...");
    ota_server_start();
    vTaskDelete(NULL);
}
#endif

uint8_t powerstate()
{
    return 1;
}

#ifdef CONFIG_RTK1010_NODE_CONSOLE_OFF
/**
 * @brief con_null
 * @return 
 */
int con_null(const char* format, va_list args)
{
    (void)format;
    (void)args;
    return 1;
}
#endif

/**
 * @brief con_udp_log
 * @param format
 * @param args
 * @return 
 */
//#define CONFIG_RTK1010_NODE_CONSOLE_UDP_HOST "192.168.0.100"
//#define CONFIG_RTK1010_NODE_CONSOLE_UDP_PORT 42007
#ifdef CONFIG_RTK1010_NODE_CONSOLE_UDP
static vprintf_like_t con_deflog = NULL;
int con_udp_log(const char* format, va_list args)
{
    static int sock = -1;
    if(sock == -1)
    {
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    }

    if( sock != -1 )
    {
        static char linebuf[512];
        int n = vsnprintf(linebuf, sizeof(linebuf) - 1, format, args);
        if(n > 0)
        {
            struct sockaddr_in dest_addr;
            dest_addr.sin_addr.s_addr = inet_addr(CONFIG_RTK1010_NODE_CONSOLE_UDP_HOST);
            dest_addr.sin_family = AF_INET;
            dest_addr.sin_port = htons(CONFIG_RTK1010_NODE_CONSOLE_UDP_PORT);
            sendto(sock, linebuf, n, MSG_DONTWAIT, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        }
    }
    return 1;
}
#endif

#ifdef CONFIG_RTK1010_NODE_CDC_ADAPTER
#error not implemented yet
#endif

/**
 * @brief 
 */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

#ifdef CONFIG_RTK1010_NODE_CONSOLE_USB_ACM
    /* switch to USB console
     */
    tinyusb_config_t tusb_cfg = { 0 }; // the configuration uses default values
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    tinyusb_config_cdcacm_t amc_cfg = { 0 }; // the configuration uses default values
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
    esp_tusb_init_console(TINYUSB_CDC_ACM_0); // log to usb
#endif

#ifdef CONFIG_RTK1010_NODE_CONSOLE_OFF
    con_deflog = esp_log_set_vprintf(con_null);
#endif

    /* init wifi
     */
    initialise_wifi();

#ifdef CONFIG_RTK1010_NODE_CONSOLE_UDP
    /* switch to UDP console
     */
    ESP_LOGI(TAG, "wait for WIFI connection ...");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, 5000 / portTICK_PERIOD_MS); // wait max 5 seconds for wifi connection
    ESP_LOGI(TAG, "switch to UDP console ...");
    con_deflog = esp_log_set_vprintf(con_udp_log);
#endif

    /* print some chip information ...
     */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "silicon revision %d, ", chip_info.revision);

    ESP_LOGI(TAG, "%dMB %s flash", spi_flash_get_chip_size() / (1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    ESP_LOGI(TAG, "heap: %d bytes free", xPortGetFreeHeapSize());

#if defined(CONFIG_PARTITION_TABLE_CUSTOM) || defined(CONFIG_PARTITION_TABLE_TWO_OTA)
    xTaskCreate(ota_server_task, "ota_server_task", 4096, NULL, 5, NULL);
#endif

    gps_init();
  
    while(1)
    {
        //ESP_LOGI(TAG, "...");
        usleep(10000000);
    }
}
