#include "sdkconfig.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

//#include <exception>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

//#include "../components/http_server/my_http_server.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "nvs_flash.h"

#include "ntrip.h"
#include <esp_http_client.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "gpgga.h"
#include "gps.h"
#include "nmea.h"
#include "parser.h"

static const char* TAG = "gps";

extern EventGroupHandle_t wifi_event_group;
#define CONNECTED_BIT BIT0;

uint8_t powerstate();

#define PORT 20000

#define KEEPALIVE_IDLE 5
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT 3

#define GPS_UART1_TXD 1
#define GPS_UART1_RXD 2
#undef GPS_UART1_INV

#ifdef RTK1010_NODE_ROVER_NTRIP_CLIENT
#define ENABLE_NTRIP_CLIENT
#endif

uint8_t gps_uart_ready = 0;
uint8_t gps_sapos_ready = 0;

#define RXBUF_SIZE 1024
uint8_t gps_rx_buffer[RXBUF_SIZE + 2];

static void gps_uart_handle();

struct
{
    struct
    {
        nmea_gpgga_s gga;
    } uart;
#ifdef ENABLE_NTRIP_CLIENT    
    struct
    {
        uint8_t needed;
        uint8_t reconnect;
        char host[48];
        uint16_t port;
        char username[32];
        char password[32];
        char mountpoint[32];
        int64_t reconnect_timeout;
        esp_http_client_handle_t http;
        int64_t gga_send_timeout;
        int64_t gga_print_timeout;
        int64_t ntrip_data_timeout;
        char gga_line[256];
        uint ntrip_rx_bytes;
    } ntrip;
#endif
    struct
    {
        int listen_sock;
        int client_sock;
    } tcpserv;
} gps_md = {};

static void gps_tcpserv_handle();
static void gps_uart_handle();

#ifdef ENABLE_NTRIP_CLIENT    
/**
 * @brief
 * @param evt
 * @return
 */
esp_err_t gps_ntrip_ev(esp_http_client_event_t* evt)
{
    switch(evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGI(TAG, "ntrip_ev() HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "ntrip_ev() HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "ntrip_ev() HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGI(TAG, "ntrip_ev() HTTP_EVENT_ON_HEADER");
        ESP_LOGI(TAG, "%.*s", evt->data_len, (char*)evt->data);
        break;
    case HTTP_EVENT_ON_DATA:
        // ESP_LOGV(TAG, "ntrip_ev() HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        gps_md.ntrip.ntrip_rx_bytes += evt->data_len;
        gps_md.ntrip.ntrip_data_timeout = esp_timer_get_time() + 30000000;
#if 0        
        if(!esp_http_client_is_chunked_response(evt->client))
        {
            ESP_LOGI(TAG, "%.*s", evt->data_len, (char*)evt->data);
        }
#endif
#if 0
        {
            ESP_LOGW(TAG, "ntrip_ev() HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        }
#endif
        if(uart_is_driver_installed(UART_NUM_1))
        {
            uart_write_bytes(UART_NUM_1, (const char*)evt->data, evt->data_len);
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGI(TAG, "ntrip_ev() HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "ntrip_ev() HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

/**
 * @brief
 * @param pvParameters
 */
static void ntrip_task(void* pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "ntrip_task()  ...");

    while(true)
    {
        int ntrip_delay = 3000;

        EventBits_t ev = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, 1 / portTICK_PERIOD_MS);
        if(ev != CONNECTED_BIT)
        {
            vTaskDelay(ntrip_delay / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "ntrip_task delay #1, retry");
            continue;
        }

        if(gps_md.ntrip.needed == 0)
        {
            vTaskDelay(ntrip_delay / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "ntrip_task delay #2, retry");
            continue;
        }

        gps_md.ntrip.gga_line[0] = 0;
        gps_md.ntrip.reconnect = 0;

        while(gps_md.ntrip.needed != 0)
        {
            if(gps_md.ntrip.reconnect_timeout > esp_timer_get_time())
            {
                vTaskDelay(ntrip_delay / portTICK_PERIOD_MS);
                ESP_LOGI(TAG, "ntrip_task delay #3, retry");
                continue;
            }

            ESP_LOGI(TAG, "ntrip_task starting ...");

            // Configure host URL
            esp_http_client_config_t config = {
                .host = gps_md.ntrip.host,
                .port = gps_md.ntrip.port,
                .method = HTTP_METHOD_GET,
            };
            config.path = (const char*)gps_md.ntrip.mountpoint;
            config.auth_type = HTTP_AUTH_TYPE_BASIC;
            config.username = (const char*)gps_md.ntrip.username;
            config.password = (const char*)gps_md.ntrip.password;
            config.event_handler = gps_ntrip_ev;

            ESP_LOGE(TAG, "ntrip_task() connect to %s:%d %s %s %s ...",config.host,config.port,config.username,config.password,config.path);

            // Initialize client
            gps_md.ntrip.http = esp_http_client_init(&config);
            esp_http_client_set_header(gps_md.ntrip.http, "Ntrip-Version", "Ntrip/2.0");
            esp_http_client_set_header(gps_md.ntrip.http, "User-Agent", "NTRIP " NTRIP_CLIENT_NAME "/2.0");
            esp_http_client_set_header(gps_md.ntrip.http, "Connection", "close");

            esp_err_t err = esp_http_client_open(gps_md.ntrip.http, 0);
            if(err != ESP_OK)
            {
                ESP_LOGE(TAG, "ntrip_task() Could not open HTTP connection: %d %s", err, esp_err_to_name(err));
                gps_md.ntrip.reconnect = 1;
                break;
            }

            int content_length = esp_http_client_fetch_headers(gps_md.ntrip.http);
            if(content_length < 0)
            {
                ESP_LOGE(TAG, "ntrip_task() Could not connect to caster: %d %s", errno, strerror(errno));
                gps_md.ntrip.reconnect = 1;
                break;
            }

            int status_code = esp_http_client_get_status_code(gps_md.ntrip.http);
            if(status_code != 200)
            {
                ESP_LOGE(TAG, "ntrip_task() Could not access mountpoint: %d", status_code);
                gps_md.ntrip.reconnect = 1;
                break;
            }

            if(!esp_http_client_is_chunked_response(gps_md.ntrip.http))
            {
                ESP_LOGE(TAG, "ntrip_task() Caster did not respond with chunked transfer encoding: content_length %d",
                    content_length);
                gps_md.ntrip.reconnect = 1;
                break;
            }
#if 0
            if(uart_is_driver_installed(UART_NUM_1))
            {
                sprintf((char*)gps_rx_buffer, "$PESP,NTRIP,SRV,CONNECTED,%s:%d,%s", gps_md.ntrip.host,
                    gps_md.ntrip.port, gps_md.ntrip.mountpoint);
                uart_write_bytes(UART_NUM_1, (const char*)gps_rx_buffer, strlen((char*)gps_rx_buffer));
            }
#endif
// RTK1010:
//$PAIR050,100*22
//$PAIR070,5*24
//$PLSC,FIXRATE,5*6C
#if 0
            if(uart_is_driver_installed(UART_NUM_1))
            {
                sprintf((char*)gps_rx_buffer, "$PAIR050,100*22\r\n");
                uart_write_bytes(UART_NUM_1, (const char*)gps_rx_buffer, strlen((char*)gps_rx_buffer));
                sprintf((char*)gps_rx_buffer, "$PAIR070,5*24\r\n");
                uart_write_bytes(UART_NUM_1, (const char*)gps_rx_buffer, strlen((char*)gps_rx_buffer));
                sprintf((char*)gps_rx_buffer, "$PLSC,FIXRATE,5*6C\r\n");
                uart_write_bytes(UART_NUM_1, (const char*)gps_rx_buffer, strlen((char*)gps_rx_buffer));
            }

#endif
            ESP_LOGI(TAG, "ntrip_task() Successfully connected (%d,%p,%d)", gps_md.ntrip.needed, gps_md.ntrip.host,
                gps_md.ntrip.reconnect);

            gps_md.ntrip.gga_send_timeout = esp_timer_get_time();
            gps_md.ntrip.ntrip_data_timeout = 0;
            gps_md.ntrip.reconnect = 0;

            while(gps_md.ntrip.needed != 0 && gps_md.ntrip.host != NULL && gps_md.ntrip.reconnect == 0)
            {
                int ret;
                char rx_buffer[512];
                // ESP_LOGI(TAG, "gps_ntrip_handle ...");

                if(gps_md.uart.gga.position_fix > 0)
                {
                    int n = strlen(gps_md.ntrip.gga_line);
                    if(n > 0)
                    {
                        if(gps_md.ntrip.gga_send_timeout < esp_timer_get_time())
                        {
                            gps_md.ntrip.gga_send_timeout = esp_timer_get_time() + 10000000;

                            ESP_LOGI(TAG, "=>NTRIP: %s", gps_md.ntrip.gga_line);
                            ret = esp_http_client_write(gps_md.ntrip.http, (char*)gps_md.ntrip.gga_line, n);
                            gps_md.ntrip.gga_line[0] = 0;
                            if(ret < 0)
                            {
                                ESP_LOGW(TAG, "ntrip_task() send error %d", ret);
                                gps_md.ntrip.reconnect = 1;
                                break;
                            }
                            gps_md.ntrip.ntrip_data_timeout = esp_timer_get_time() + 10000000;
                        }
                    }

                    if(gps_md.ntrip.ntrip_data_timeout != 0)
                    {
                        if(gps_md.ntrip.ntrip_data_timeout < esp_timer_get_time())
                        {
                            ESP_LOGI(TAG, "=>NTRIP: TIMEOUT, reconnect");
                            gps_md.ntrip.ntrip_data_timeout = 0;
                            gps_md.ntrip.reconnect = 1;
                        }
                    }
                }

                ret = esp_http_client_read(gps_md.ntrip.http, rx_buffer, sizeof(rx_buffer));
                if(ret < 0)
                {
                    ESP_LOGW(TAG, "ntrip_task() read error %d", ret);
                    gps_md.ntrip.reconnect = 1;
                    vTaskDelay(10 / portTICK_PERIOD_MS);
                    break;
                }
            }

            gps_md.ntrip.reconnect_timeout = esp_timer_get_time() + 10000000;

            ESP_LOGI(TAG, "ntrip_task() Disconnected");
            if(gps_md.ntrip.http != NULL)
            {
                esp_http_client_close(gps_md.ntrip.http);
                esp_http_client_cleanup(gps_md.ntrip.http);
                gps_md.ntrip.http = NULL;
            }
        }
    }
}
#endif

/**
 * @brief
 */
void gps_ntrip_start()
{
#ifdef ENABLE_NTRIP_CLIENT    
    gps_md.ntrip.needed = 1;
#endif    
}

/**
 * @brief
 */
void gps_ntrip_stop()
{
#ifdef ENABLE_NTRIP_CLIENT    
    gps_md.ntrip.needed = 0;
#endif
}

/**
 * @brief
 */
static void gps_tcpserv_stop()
{
    if(gps_md.tcpserv.client_sock != 0)
    {
        close(gps_md.tcpserv.client_sock);
        gps_md.tcpserv.client_sock = 0;
    }
    if(gps_md.tcpserv.listen_sock != 0)
    {
        close(gps_md.tcpserv.listen_sock);
        gps_md.tcpserv.listen_sock = 0;
    }
}

/**
 * @brief
 */
static void gps_tcpserv_start()
{
    if(gps_md.tcpserv.listen_sock == 0)
    {
        ESP_LOGI(TAG, "gps_tcpserv_start ...");

        struct sockaddr_in6 destAddr;
        int ip_protocol;
        int addr_family;
        char addr_str[128];
        bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);

        gps_md.tcpserv.listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if(gps_md.tcpserv.listen_sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            gps_tcpserv_stop();
            return;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(gps_md.tcpserv.listen_sock, (struct sockaddr*)&destAddr, sizeof(destAddr));
        if(err != 0)
        {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            gps_tcpserv_stop();
            return;
        }

        err = listen(gps_md.tcpserv.listen_sock, 1);
        if(err != 0)
        {
            ESP_LOGE(TAG, "Error during listen: errno %d", errno);
            gps_tcpserv_stop();
            return;
        }

        ESP_LOGI(TAG, "gps_tcpserv_start ... ok");
    }
}
/**
 * @brief
 */
static void gps_tcpserv_handle()
{
    if(gps_md.tcpserv.listen_sock == 0)
    {
        gps_tcpserv_start();
    }

    if(gps_md.tcpserv.listen_sock != 0)
    {
        if(gps_md.tcpserv.client_sock == 0)
        {
            // ESP_LOGI(TAG, "gps_tcpserv_handle ...");

            fd_set rfds;
            struct timeval tv;
            int retval;
            FD_ZERO(&rfds);
            FD_SET(gps_md.tcpserv.listen_sock, &rfds);
            tv.tv_sec = 0;
            tv.tv_usec = 1000;
            retval = select(gps_md.tcpserv.listen_sock + 1, &rfds, NULL, NULL, &tv);

            if(retval == -1)
            {
                perror("select()");
                //gps_tcpserv_stop();
            }
            else if(retval)
            {
                ESP_LOGI(TAG, "gps_tcpserv_handle ... ...");

                struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
                uint addrLen = sizeof(sourceAddr);
                gps_md.tcpserv.client_sock =
                    accept(gps_md.tcpserv.listen_sock, (struct sockaddr*)&sourceAddr, &addrLen);

                if(gps_md.tcpserv.client_sock > 0)
                {
                    int keepAlive = 1;
                    int keepIdle = KEEPALIVE_IDLE;
                    int keepInterval = KEEPALIVE_INTERVAL;
                    int keepCount = KEEPALIVE_COUNT;
                    setsockopt(gps_md.tcpserv.client_sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
                    setsockopt(gps_md.tcpserv.client_sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
                    setsockopt(gps_md.tcpserv.client_sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
                    setsockopt(gps_md.tcpserv.client_sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

                    ESP_LOGI(TAG, "gps_tcpserv_handle ... connected");
                }
            }
        }
    }

    if(gps_md.tcpserv.client_sock != 0)
    {
        int len = 0;
        len = recv(gps_md.tcpserv.client_sock, gps_rx_buffer, RXBUF_SIZE, MSG_DONTWAIT);
#if 0        
        if(len < 0)
        {
            close(gps_md.tcpserv.client_sock);
            gps_md.tcpserv.client_sock = 0;
        }
        else 
#endif            
        if(len > 0)
        {
            if(uart_is_driver_installed(UART_NUM_1))
            {
                uart_write_bytes(UART_NUM_1, (const char*)gps_rx_buffer, len);
            }
        }
    }
}

/**
 * @brief
 * @param buflen
 * @param buf
 */
static void gps_handle_nmea(int buflen, const char* buf)
{
    static char linebuf[512];
    static int linebuf_used = 0;
    int i;
    for(i = 0; i < buflen; i++)
    {
        if(linebuf_used >= sizeof(linebuf))
        {
            linebuf_used = 0;
        }
        linebuf[linebuf_used] = buf[i];
        if((((unsigned char)buf[i]) >= 32) || (buf[i] == '\r'))
        {
            linebuf_used++;
        }
        else
        {
            if(linebuf_used > 1)
            {
                char linebuf_bak[sizeof(linebuf)] = {};
                // ESP_LOGW(TAG,"F9P: %s", linebuf);

                strncpy(linebuf_bak, linebuf, linebuf_used + 1);
                if(linebuf_used >= 3)
                {
                    if(linebuf[0] == '$' && linebuf[1] == 'G' && linebuf[2] == 'N')
                    {
                        linebuf[2] = 'P'; // check_checksum=0
                    }
                }
                nmea_s* p = nmea_parse(linebuf, linebuf_used + 1, 0);
                if(p != NULL)
                {
                    // ESP_LOGW(TAG, "%d %d %s", p->errors,p->type,linebuf);
                    if(NMEA_GPGGA == p->type)
                    {
                        gps_md.uart.gga = *((nmea_gpgga_s*)p);

#ifdef ENABLE_NTRIP_CLIENT    
                        if(gps_md.ntrip.gga_print_timeout < esp_timer_get_time())
                        {
                            gps_md.ntrip.gga_print_timeout = esp_timer_get_time() + 3000000;
                            ESP_LOGW(TAG, "fix=%d N=%d pos=(%d %f %d %f) ntrip-rx=%d", gps_md.uart.gga.position_fix,
                                gps_md.uart.gga.n_satellites, gps_md.uart.gga.latitude.degrees,
                                gps_md.uart.gga.latitude.minutes, gps_md.uart.gga.longitude.degrees,
                                gps_md.uart.gga.longitude.minutes, gps_md.ntrip.ntrip_rx_bytes);
                        }
#endif                        

                        if(gps_md.uart.gga.position_fix > 0)
                        {
#ifdef ENABLE_NTRIP_CLIENT    
                            strncpy(gps_md.ntrip.gga_line, linebuf_bak, linebuf_used + 1);
                            gps_md.ntrip.gga_line[linebuf_used + 1] = 0;
#endif                            
                            // ESP_LOGW(TAG, "XXX %s", gps_md.ntrip.gga_line);
                            gps_ntrip_start();
                        }
                        else
                        {
                            gps_ntrip_stop();
                        }
                    }
                    nmea_free(p);
                }
                else
                {
                    // ESP_LOGW(TAG, "error %s", linebuf);
                }
            }
            linebuf_used = 0;
        }
        if(linebuf_used == 1 && linebuf[0] != '$')
        {
            linebuf_used = 0;
        }
    }
}

/**
 * @brief
 */
static void gps_uart_stop()
{
    if(uart_is_driver_installed(UART_NUM_1))
    {
        ESP_LOGI(TAG, "gps_uart_stop");
        uart_driver_delete(UART_NUM_1);
        memset((void*)&gps_md.uart, 0, sizeof(gps_md.uart));
    }
}

/**
 * @brief
 */
static void gps_uart_start()
{
    if(!uart_is_driver_installed(UART_NUM_1))
    {
        ESP_LOGI(TAG, "gps_uart_start");

        memset((void*)&gps_md.uart, 0, sizeof(gps_md.uart));

        /* Configure parameters of an UART driver,
         * communication pins and install the driver */
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_REF_TICK,
        };

        int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

        uart_driver_install(UART_NUM_1, RXBUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags);
        uart_param_config(UART_NUM_1, &uart_config);
        uart_set_pin(UART_NUM_1, GPS_UART1_TXD, GPS_UART1_RXD, -1, -1);
#ifdef GPS_UART1_INV
        uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
#endif

        uart_flush(UART_NUM_1);
    }
}
/**
 * @brief
 */
static void gps_uart_handle()
{
    if(uart_is_driver_installed(UART_NUM_1))
    {
        // ESP_LOGI(TAG, "gps_uart_handle");
        int tout = (1 + (RXBUF_SIZE / (10 * 3)));
        if(tout > 50)
            tout = 20;
        int len = uart_read_bytes(UART_NUM_1, gps_rx_buffer, RXBUF_SIZE, tout / portTICK_RATE_MS);
        if(len > 0)
        {
            if(gps_md.tcpserv.client_sock != 0)
            {
                send(gps_md.tcpserv.client_sock, gps_rx_buffer, len, 0);
            }
            gps_rx_buffer[len] = 0;
            //ESP_LOGW(TAG, "tx %d %s", len, gps_rx_buffer);
            gps_handle_nmea(len, (const char*)gps_rx_buffer);
        }
    }
}

/**
 * @brief
 * @param pvParameters
 */
static void gps_task(void* pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "gps_task TXD=%d RXD=%d", GPS_UART1_TXD, GPS_UART1_RXD);

    nmea_load_parsers();

    while(true)
    {
        const int gps_wait = 1000;

        ESP_LOGI(TAG, "Wait until power is on ... ");
        while(powerstate() == 0)
        {
            gps_uart_stop();
            vTaskDelay(gps_wait / portTICK_PERIOD_MS);
        }

        ESP_LOGI(TAG, "Starting ... ");
        gps_uart_start();
        gps_tcpserv_start();

        while(true)
        {
            if(powerstate() == 0)
            {
                break; /* stop GPS when power down */
            }

            gps_uart_handle();
            gps_tcpserv_handle();
            // gps_ntrip_handle();
        }

        gps_ntrip_stop();
        gps_tcpserv_stop();
        gps_uart_stop();
        vTaskDelete(NULL);
    }
}

/**
 * @brief
 */
void gps_init()
{
#ifdef ENABLE_NTRIP_CLIENT    
    strcpy(gps_md.ntrip.host, CONFIG_NTRIP_HOST);
    gps_md.ntrip.port = CONFIG_NTRIP_PORT;
    strcpy(gps_md.ntrip.username, CONFIG_NTRIP_USERNAME);
    strcpy(gps_md.ntrip.password, CONFIG_NTRIP_PASSWORD);
    strcpy(gps_md.ntrip.mountpoint, CONFIG_NTRIP_MOUNTPOINT);

    ESP_LOGI(TAG, "gps_init host=%s:%d user=%s passwd=%s mountpoint=%s", gps_md.ntrip.host, gps_md.ntrip.port,
        gps_md.ntrip.username, gps_md.ntrip.password, gps_md.ntrip.mountpoint);

    if(gps_md.ntrip.port != 0 && strlen(gps_md.ntrip.host) != 0 && strlen(gps_md.ntrip.username) != 0 &&
        strlen(gps_md.ntrip.password) != 0 && strlen(gps_md.ntrip.mountpoint) != 0)
    {
        xTaskCreate(ntrip_task, "ntrip_task", 4096, NULL, 0, NULL);
    }
#endif    
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 0, NULL);
}
/**
 * @brief
 */
void gps_exit()
{
}
