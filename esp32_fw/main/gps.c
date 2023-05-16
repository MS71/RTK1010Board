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

#include "driver/gpio.h"
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

#if defined(CONFIG_RTK1010_NODE_PASS_TO_CDC)
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#endif

static const char* TAG = "gps";

extern EventGroupHandle_t wifi_event_group;
#define CONNECTED_BIT BIT0

uint8_t powerstate();

#define TCPSERV_PORT 20000

#define KEEPALIVE_IDLE 5
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT 3

#define GPS_UART1_TXD 1
#define GPS_UART1_RXD 2
#undef GPS_UART1_INV

uint8_t gps_uart_ready = 0;
uint8_t gps_sapos_ready = 0;

uint8_t gps_rx_buffer[256 + 2];
uint8_t gps_tx_buffer[256 + 2];

static void gps_uart_handle();

struct
{
    struct
    {
        uint8_t initflag;
        uint rx_bytes;
        uint tx_bytes;
        uint rx_errors;
        int position_fix;
    } uart;
    struct
    {
#ifdef CONFIG_RTK1010_NODE_ROVER_NTRIP_CLIENT
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
#endif
        int64_t gga_print_timeout;
        uint ntrip_rx_bytes;
#ifdef CONFIG_RTK1010_NODE_ROVER_NTRIP_CLIENT
        int64_t ntrip_data_timeout;
        char gga_line[256];
#endif
    } ntrip;
    struct
    {
        int listen_sock;
        int client_sock;
    } tcpserv;
#ifdef CONFIG_RTK1010_NODE_BASE_RTCM_SERVER
    struct
    {
        int listen_sock;
        int client_sock;
    } rtcmtcpserv;
#endif
#ifdef CONFIG_RTK1010_NODE_ROVER_RTCM_CLIENT
    struct
    {
        int client_sock;
        int64_t reconnect_timeout;
    } rtcmtcpclient;
#endif
} gps_md = {};

static void gps_tcpserv_handle();
static void gps_uart_handle();

typedef struct
{
    uint8_t msg_buf[512];
    uint16_t msg_idx;
    enum
    {
        STATE_IDLE = 0,
        STATE_NMEA_START,
        STATE_RTCM3_LEN0,
        STATE_RTCM3_LEN1,
        STATE_RTCM3_BODY,
    } state;
    uint16_t rtcm_len;
} GPSLoggerType;
GPSLoggerType gps_logger[1] = {};

double A = 6378137.0, B = 6356752.3142;

double deg2rad(double r)
{
    return (M_PI * r / 180.0);
}

void lla2ecef(double* lla, double* ecef)
{
    double N;
    lla[0] = deg2rad(lla[0]);
    lla[1] = deg2rad(lla[1]);
    N = pow(A, 2) / sqrt(pow(A, 2) * pow(cos(lla[0]), 2) + pow(B, 2) * pow(sin(lla[0]), 2));
    ecef[0] = (N + lla[2]) * cos(lla[0]) * cos(lla[1]);
    ecef[1] = (N + lla[2]) * cos(lla[0]) * sin(lla[1]);
    ecef[2] = (pow(B, 2) / pow(A, 2) * N + lla[2]) * sin(lla[0]);
    return;
}

double ddmmtodd(double ddm) 
{
    double deg = floor(ddm / 100);
    double min = ddm - deg * 100;
    return deg + min / 60;
}

/**
 * @brief calculate a RTCM message parity value
 * @param Crc
 * @param Size
 * @param Buffer
 * @return
 */
uint32_t rtcm_parity(uint32_t Crc, uint32_t Size, uint8_t* Buffer) // sourcer32@gmail.com
{
    static const uint32_t crctab[] = { // Nibble lookup for Qualcomm CRC-24Q
        0x00000000, 0x01864CFB, 0x038AD50D, 0x020C99F6, 0x0793E6E1, 0x0615AA1A, 0x041933EC, 0x059F7F17, 0x0FA18139,
        0x0E27CDC2, 0x0C2B5434, 0x0DAD18CF, 0x083267D8, 0x09B42B23, 0x0BB8B2D5, 0x0A3EFE2E
    };

    while(Size--)
    {
        Crc ^= (uint32_t)*Buffer++ << 16; // Apply byte
        // Process 8-bits, 4 at a time, or 2 rounds
        Crc = (Crc << 4) ^ crctab[(Crc >> 20) & 0x0F];
        Crc = (Crc << 4) ^ crctab[(Crc >> 20) & 0x0F];
    }

    return (Crc & 0xFFFFFF); // Mask to 24-bit, as above optimized for 32-bit
}

/**
 * @brief
 * @param rtcm_type
 * @param rtcm_bytes
 * @param rtcm_msg
 */
void rtcm_message_filter(uint32_t rtcm_type, uint16_t rtcm_bytes, uint8_t* rtcm_msg)
{
    if(rtcm_bytes >= 8)
    {
#if 0
        if((rtcm_type == 1005) || (rtcm_type == 1074) || (rtcm_type == 1084) || (rtcm_type == 1094) ||
            (rtcm_type == 1114) || (rtcm_type == 1124))
#endif
#if 1
        if((rtcm_type == 1005) || (rtcm_type == 1074) || (rtcm_type == 1084) || (rtcm_type == 1094) ||
            (rtcm_type == 1114) || (rtcm_type == 1124))
#endif
        {
#if 0
            ESP_LOGW(TAG, "rtcm_filter() RTCM type=%d %02x%02x%02x%02x%02x...%02x%02x", 
            rtcm_type,
            rtcm_msg[0],rtcm_msg[1],rtcm_msg[2],rtcm_msg[3],rtcm_msg[4],
            rtcm_msg[rtcm_bytes-2],rtcm_msg[rtcm_bytes-1]);
#endif
            uart_write_bytes(UART_NUM_1, (const char*)rtcm_msg, rtcm_bytes);
            gps_md.uart.tx_bytes += rtcm_bytes;
        }
    }
}

/**
 * @brief
 * @param l
 * @param data_len
 * @param data
 */
void rtcm_message_parser(GPSLoggerType* l, uint data_len, uint8_t* data)
{
    // https://singularxyz.com/471.html
    int i;
    for(i = 0; i < data_len; i++)
    {
        if(l->state == STATE_IDLE)
        {
            if((char)data[i] == '$')
            {
                l->state = STATE_NMEA_START;
                l->msg_idx = 0;
                l->msg_buf[l->msg_idx++] = data[i];
                l->msg_buf[l->msg_idx] = 0;
            }
            else if(data[i] == 0xd3)
            {
                l->state = STATE_RTCM3_LEN0;
                l->msg_idx = 0;
                l->msg_buf[l->msg_idx++] = data[i];
                l->msg_buf[l->msg_idx] = 0;
            }
        }
        else if(l->state == STATE_NMEA_START)
        {
            if(data[i] < 32)
            {
                ESP_LOGI(TAG, "rtcm_message_parser() NMEA %d [%s]", l->msg_idx, (char*)(l->msg_buf));
                l->state = STATE_IDLE;
            }
            else if(l->msg_idx < (sizeof(l->msg_buf) - 1))
            {
                l->msg_buf[l->msg_idx++] = data[i];
                l->msg_buf[l->msg_idx] = 0;
            }
        }
        else if(l->state == STATE_RTCM3_LEN0)
        {
            l->msg_buf[l->msg_idx++] = data[i];
            l->msg_buf[l->msg_idx] = 0;
            l->rtcm_len = data[i];
            l->state = STATE_RTCM3_LEN1;
        }
        else if(l->state == STATE_RTCM3_LEN1)
        {
            l->msg_buf[l->msg_idx++] = data[i];
            l->msg_buf[l->msg_idx] = 0;
            l->rtcm_len = ((l->rtcm_len << 8 | data[i]) & 0x3ff) + 3 + 3;
            if(l->rtcm_len > 6)
            {
                l->state = STATE_RTCM3_BODY;
            }
            else
            {
                l->state = STATE_IDLE;
            }
        }
        else if(l->state == STATE_RTCM3_BODY)
        {
            if(l->msg_idx < (sizeof(l->msg_buf) - 10))
            {
                l->msg_buf[l->msg_idx] = data[i];
                l->msg_buf[l->msg_idx + 1] = 0;
            }
            else
            {
                ESP_LOGI(TAG, "rtcm_message_parser() error #3 state=%d", l->state);
                l->state = STATE_IDLE;
            }
            l->msg_idx++;

            if(l->msg_idx >= l->rtcm_len)
            {
                uint32_t crc = rtcm_parity(0x00000000, l->rtcm_len, l->msg_buf);
                uint32_t type = ((l->msg_buf[3] << 4) | l->msg_buf[4] >> 4);
                // ESP_LOGI(TAG, "rtcm_message_parser() RTCM len=0x%04x crc=0x%08x type=%d", l->rtcm_len - 6, crc,
                // type);
                l->state = STATE_IDLE;
                if(crc == 0)
                {
                    rtcm_message_filter(type, l->rtcm_len, l->msg_buf);
                }
            }
        }
        else
        {
            ESP_LOGI(TAG, "rtcm_message_parser() error #1 state=%d", l->state);
            l->state = STATE_IDLE;
        }
    }
}

/**
 * @brief
 * @param cmd
 * @return
 */
int gps_nmea_csum(const char* cmd)
{
    if(cmd[0] == '$')
    {
        uint8_t chk = 0;
        int n = 0;
        {
            const char* p = &cmd[1];

            /* While current char isn't '*' or sentence ending (newline) */
            while('*' != *p && '\0' != *p)
            {
                chk ^= (uint8_t)*p;
                p++;
                n++;
            }
        }
        if(cmd[n + 1] == '*')
        {
            return chk;
        }
    }
    return 0;
}

/**
 * @brief
 * @param line
 * @return
 */
int gps_nmea_ok(char* line)
{
    if(line != NULL)
    {
        if(line[0] == '$')
        {
            char* pcsum = strstr(line, "*");
            if(pcsum != NULL)
            {
                unsigned int csum = 0;
                if(sscanf(pcsum, "*%02x", &csum) == 1)
                {
                    if(csum == gps_nmea_csum(line))
                    {
                        // ESP_LOGW(TAG, "gps_nmea_ok 0x%02x 0x%02x", csum, gps_nmea_csum(line));
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
}

/**
 * @brief
 * @param line
 * @param idx
 * @return
 */
int gps_nmea_get_int(char* line, int idx, int def)
{
    char* s = strdup(line);
    int i = 0;
    char* token;
    char* saveptr = NULL;
    token = strtok_r(s, ",", &saveptr);
    while(token)
    {
        if(i == idx)
        {
            int r = strtol(token, NULL, 10);
            free(s);
            return r;
        }
        token = strtok_r(NULL, ",", &saveptr);
        i++;
    }
    free(s);
    return def;
}

/**
 * @brief
 * @param line
 * @param idx
 * @return
 */
float gps_nmea_get_float(char* line, int idx)
{
    char* s = strdup(line);
    int i = 0;
    char* token;
    char* saveptr = NULL;
    token = strtok_r(s, ",", &saveptr);
    while(token)
    {
        if(i == idx)
        {
            float r;
            if(sscanf(token, "%f", &r) == 1)
            {
                free(s);
                return r;
            }
        }
        token = strtok_r(NULL, ",", &saveptr);
        i++;
    }
    free(s);
    return 0;
}

/**
 * @brief
 * @param cmd
 */
void gps_nmea_send(const char* cmd)
{
    uint8_t chk = gps_nmea_csum(cmd);
    {
        //$PLSC,VER*61<CR><LF>
        if(uart_is_driver_installed(UART_NUM_1))
        {
            char end[5] = {};
            sprintf(end, "%02X\r\n", chk);
            uart_write_bytes(UART_NUM_1, (const char*)cmd, strlen(cmd));
            uart_write_bytes(UART_NUM_1, (const char*)end, 4);
            ESP_LOGI(TAG, "gps_nmea_send(%s%02X)", cmd, chk);
        }
    }
}

#ifdef CONFIG_RTK1010_NODE_ROVER_NTRIP_CLIENT
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
            rtcm_message_parser(&gps_logger[0], evt->data_len, (uint8_t*)evt->data);
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

            ESP_LOGE(TAG, "ntrip_task() connect to %s:%d %s %s %s ...", config.host, config.port, config.username,
                config.password, config.path);

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

                if(gps_md.uart.position_fix > 0)
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
#ifdef CONFIG_RTK1010_NODE_ROVER_NTRIP_CLIENT
void gps_ntrip_start()
{
    gps_md.ntrip.needed = 1;
}
#endif

/**
 * @brief
 */
#ifdef CONFIG_RTK1010_NODE_ROVER_NTRIP_CLIENT
void gps_ntrip_stop()
{
    gps_md.ntrip.needed = 0;
}
#endif

#ifdef CONFIG_RTK1010_NODE_ROVER_RTCM_CLIENT
/**
 * @brief
 */
static void gps_rtcmtcpclient_stop()
{
    if(gps_md.rtcmtcpclient.client_sock != 0)
    {
        close(gps_md.rtcmtcpclient.client_sock);
        gps_md.rtcmtcpclient.client_sock = 0;
    }
}

/**
 * @brief
 */
static void gps_rtcmtcpclient_start()
{
    if(gps_md.rtcmtcpclient.reconnect_timeout > esp_timer_get_time())
    {
        return;
    }

    if(gps_md.rtcmtcpclient.client_sock == 0)
    {
        char host_ip[] = CONFIG_RTCM_HOST;
        ESP_LOGI(TAG, "gps_rtcmtcpclient_start %s:%d ...", host_ip, CONFIG_RTCM_PORT);

        int ip_protocol = 0;
        int addr_family = 0;
#if 0
        struct sockaddr_in6 destAddr;
        char addr_str[128];
        bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
        inet6_aton(host_ip, &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(CONFIG_RTCM_PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#else
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(host_ip);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(CONFIG_RTCM_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#endif

        gps_md.rtcmtcpclient.reconnect_timeout = esp_timer_get_time() + 10000000;

        gps_md.rtcmtcpclient.client_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if(gps_md.rtcmtcpclient.client_sock < 0)
        {
            ESP_LOGE(TAG, "gps_rtcmtcpclient_start unable to create socket: errno %d", errno);
            gps_rtcmtcpclient_stop();
            return;
        }

        // fcntl(gps_md.rtcmtcpclient.client_sock, F_SETFL, O_NONBLOCK);
        int err = connect(gps_md.rtcmtcpclient.client_sock, (struct sockaddr*)&destAddr, sizeof(struct sockaddr_in6));
        if(err != 0)
        {
            ESP_LOGE(TAG, "gps_rtcmtcpclient_start Socket unable to connect: errno %d", errno);
            gps_rtcmtcpclient_stop();
            return;
        }
        // gps_md.rtcmtcpclient.reconnect_timeout = 0;

        ESP_LOGI(TAG, "gps_rtcmtcpclient_start ... ok");
    }
}

/**
 * @brief
 */
static void gps_rtcmtcpclient_handle()
{
    if(gps_md.rtcmtcpclient.client_sock == 0)
    {
        gps_rtcmtcpclient_start();
    }

    if(gps_md.rtcmtcpclient.client_sock != 0)
    {
        int len = 0;
        len = recv(gps_md.rtcmtcpclient.client_sock, gps_tx_buffer, sizeof(gps_tx_buffer) - 2, MSG_DONTWAIT);
        if(len > 0)
        {
            // ESP_LOGE(TAG,"gps_rtcmtcpclient_handle TX.GPS:%s",gps_tx_buffer);
            if(uart_is_driver_installed(UART_NUM_1))
            {
#if 0                
                uart_write_bytes(UART_NUM_1, (const char*)gps_tx_buffer, len);
                gps_md.uart.tx_bytes += len;
#else                
                rtcm_message_parser(&gps_logger[0], len, (uint8_t*)gps_tx_buffer);
#endif

                gps_md.ntrip.ntrip_rx_bytes += len;
            }
        }
        else if(len < 0 && errno != EAGAIN)
        {
            ESP_LOGI(TAG, "gps_rtcmtcpclient_handle ... disconnected errno=%d", errno);
            close(gps_md.rtcmtcpclient.client_sock);
            gps_md.rtcmtcpclient.client_sock = 0;
        }
    }
}
#endif

#ifdef CONFIG_RTK1010_NODE_BASE_RTCM_SERVER
/**
 * @brief
 */
static void gps_rtcmtcpserv_stop()
{
    if(gps_md.rtcmtcpserv.client_sock != 0)
    {
        close(gps_md.rtcmtcpserv.client_sock);
        gps_md.rtcmtcpserv.client_sock = 0;
    }
    if(gps_md.rtcmtcpserv.listen_sock != 0)
    {
        close(gps_md.rtcmtcpserv.listen_sock);
        gps_md.rtcmtcpserv.listen_sock = 0;
    }
}

/**
 * @brief
 */
static void gps_rtcmtcpserv_start()
{
    if(gps_md.rtcmtcpserv.listen_sock == 0)
    {
        ESP_LOGI(TAG, "gps_rtcmtcpserv_start ...");

        struct sockaddr_in6 destAddr;
        int ip_protocol;
        int addr_family;
        char addr_str[128];
        bzero(&destAddr.sin6_addr.un, sizeof(destAddr.sin6_addr.un));
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(CONFIG_RTCM_PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);

        gps_md.rtcmtcpserv.listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if(gps_md.rtcmtcpserv.listen_sock < 0)
        {
            ESP_LOGE(TAG, "gps_rtcmtcpserv_start unable to create socket: errno %d", errno);
            gps_rtcmtcpserv_stop();
            return;
        }

        int err = bind(gps_md.rtcmtcpserv.listen_sock, (struct sockaddr*)&destAddr, sizeof(destAddr));
        if(err != 0)
        {
            ESP_LOGE(TAG, "gps_rtcmtcpserv_start socket unable to bind: errno %d", errno);
            gps_rtcmtcpserv_stop();
            return;
        }

        err = listen(gps_md.rtcmtcpserv.listen_sock, 1);
        if(err != 0)
        {
            ESP_LOGE(TAG, "gps_rtcmtcpserv_start Error during listen: errno %d", errno);
            gps_rtcmtcpserv_stop();
            return;
        }

        ESP_LOGI(TAG, "gps_rtcmtcpserv_start ... ok");
    }
}

/**
 * @brief
 */
static void gps_rtcmtcpserv_handle()
{
    if(gps_md.rtcmtcpserv.listen_sock == 0)
    {
        gps_rtcmtcpserv_start();
    }

    if(gps_md.rtcmtcpserv.listen_sock != 0)
    {
        if(gps_md.rtcmtcpserv.client_sock == 0)
        {
            // ESP_LOGI(TAG, "gps_tcpserv_handle ...");

            fd_set rfds;
            struct timeval tv;
            int retval;
            FD_ZERO(&rfds);
            FD_SET(gps_md.rtcmtcpserv.listen_sock, &rfds);
            tv.tv_sec = 0;
            tv.tv_usec = 1000;
            retval = select(gps_md.rtcmtcpserv.listen_sock + 1, &rfds, NULL, NULL, &tv);

            if(retval == -1)
            {
                perror("select()");
                // gps_tcpserv_stop();
            }
            else if(retval)
            {
                ESP_LOGI(TAG, "gps_rtcmtcpserv_handle ... ...");

                struct sockaddr_in6 sourceAddr; // Large enough for both IPv4 or IPv6
                uint addrLen = sizeof(sourceAddr);
                gps_md.rtcmtcpserv.client_sock =
                    accept(gps_md.rtcmtcpserv.listen_sock, (struct sockaddr*)&sourceAddr, &addrLen);

                if(gps_md.rtcmtcpserv.client_sock > 0)
                {
                    int keepAlive = 1;
                    int keepIdle = KEEPALIVE_IDLE;
                    int keepInterval = KEEPALIVE_INTERVAL;
                    int keepCount = KEEPALIVE_COUNT;
                    setsockopt(gps_md.rtcmtcpserv.client_sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
                    setsockopt(gps_md.rtcmtcpserv.client_sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
                    setsockopt(gps_md.rtcmtcpserv.client_sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
                    setsockopt(gps_md.rtcmtcpserv.client_sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
                    setsockopt(gps_md.rtcmtcpserv.client_sock, IPPROTO_TCP, TCP_NODELAY, (int[]) { 1 }, sizeof(int));
                    // send ok msg
                    const char ok[] = "\r\nICY 200 OK\r\n";
                    send(gps_md.rtcmtcpserv.client_sock, &ok, strlen(ok), 0);
                    ESP_LOGI(TAG, "gps_rtcmtcpserv_handle ... connected");
                }
            }
        }
    }

    if(gps_md.rtcmtcpserv.client_sock != 0)
    {
        int len = 0;
        len = recv(gps_md.rtcmtcpserv.client_sock, gps_tx_buffer, sizeof(gps_tx_buffer) - 2, MSG_DONTWAIT);
        if(len > 0)
        {
#if 0
            if(uart_is_driver_installed(UART_NUM_1))
            {
                //ESP_LOGE(TAG,"TX.GPS:%s",gps_tx_buffer);
                uart_write_bytes(UART_NUM_1, (const char*)gps_tx_buffer, len);
                gps_md.uart.tx_bytes += len;
            }
#endif
        }
        else if(len < 0 && errno != EAGAIN)
        {
            ESP_LOGI(TAG, "gps_rtcmtcpserv_handle ... disconnected errno=%d", errno);
            close(gps_md.rtcmtcpserv.client_sock);
            gps_md.rtcmtcpserv.client_sock = 0;
        }
    }
}
#endif

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
        destAddr.sin6_port = htons(TCPSERV_PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);

        gps_md.tcpserv.listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if(gps_md.tcpserv.listen_sock < 0)
        {
            ESP_LOGE(TAG, "gps_tcpserv_start Unable to create socket: errno %d", errno);
            gps_tcpserv_stop();
            return;
        }

        int err = bind(gps_md.tcpserv.listen_sock, (struct sockaddr*)&destAddr, sizeof(destAddr));
        if(err != 0)
        {
            ESP_LOGE(TAG, "gps_tcpserv_start Socket unable to bind: errno %d", errno);
            gps_tcpserv_stop();
            return;
        }

        err = listen(gps_md.tcpserv.listen_sock, 1);
        if(err != 0)
        {
            ESP_LOGE(TAG, "gps_tcpserv_start Error during listen: errno %d", errno);
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
                // gps_tcpserv_stop();
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
                    setsockopt(gps_md.tcpserv.client_sock, IPPROTO_TCP, TCP_NODELAY, (int[]) { 1 }, sizeof(int));
                    ESP_LOGI(TAG, "gps_tcpserv_handle ... connected");
                }
            }
        }
    }

    if(gps_md.tcpserv.client_sock != 0)
    {
        int len = 0;
        len = recv(gps_md.tcpserv.client_sock, gps_tx_buffer, sizeof(gps_tx_buffer) - 2, MSG_DONTWAIT);
        if(len > 0)
        {
            if(uart_is_driver_installed(UART_NUM_1))
            {
                // ESP_LOGE(TAG,"TX.GPS:%s",gps_tx_buffer);
                uart_write_bytes(UART_NUM_1, (const char*)gps_tx_buffer, len);
                gps_md.uart.tx_bytes += len;
            }
        }
        else if(len < 0 && errno != EAGAIN)
        {
            ESP_LOGI(TAG, "gps_tcpserv_handle ... disconnected errno=%d", errno);
            close(gps_md.tcpserv.client_sock);
            gps_md.tcpserv.client_sock = 0;
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
    static char linebuf[256];
    static char linebuf_bak[256];
    static int linebuf_used = 0;
    int i;
    for(i = 0; i < buflen; i++)
    {
        if(linebuf_used >= (sizeof(linebuf) - 5))
        {
            ESP_LOGW(TAG, "gps_handle_nmea() overflow");
            linebuf_used = 0;
            gps_md.uart.rx_errors++;
        }
        linebuf[linebuf_used] = buf[i];
        if(linebuf_used == 0 && linebuf[0] != '$')
        {
            // start with $
        }
        else if((((unsigned char)buf[i]) >= 32) /*|| (buf[i] == '\r') || (buf[i] == '\n')*/)
        {
            linebuf_used++;
        }
        else
        {
            if(linebuf_used > 1)
            {
                linebuf[linebuf_used++] = '\r';
                linebuf[linebuf_used++] = '\n';
                linebuf[linebuf_used] = 0;
                    //ESP_LOGE(TAG, "RX.RTK-1010: gps_md.uart.initflag=%d", gps_md.uart.initflag);
                if(gps_md.uart.initflag == 2 && linebuf_used > 4 && (strstr(linebuf, "$PAIR00") == &linebuf[0]) )
                { // WARM START
                    gps_md.uart.initflag = 3;
                    linebuf[linebuf_used - 2] = 0;
                    linebuf[linebuf_used - 1] = 0;
                    ESP_LOGE(TAG, "RX.RTK-1010: {%s} !!! RESTART !!! init sequence ...", linebuf);
                    gps_nmea_send("$PLSC,VER*"); // request firmware version
#if defined(CONFIG_RTK1010_NODE_ROVER_NTRIP_CLIENT) || defined(CONFIG_RTK1010_NODE_ROVER_RTCM_CLIENT)
                    //gps_nmea_send("$PLSC,FIXRATE,10*"); // Set fixrate to 10Hz
                    //gps_nmea_send("$PLSC,MCBASE,0*"); // Set up the board as a rover(default)
                    // gps_nmea_send("$PAIR050,100*"); // Set position fix intervall to 10Hz
                    // gps_nmea_send("$PLSC,SETBASEXYZ,3856.062429080888,690.0109524095845,5016.542125862716*");
#elif defined(CONFIG_RTK1010_NODE_BASE_RTCM_SERVER)
                    // https://dominoc925-pages.appspot.com/mapplets/cs_ecef.html
#ifdef CONFIG_RTCM_SERVER_BASEXYZ
                    if(strlen(CONFIG_RTCM_SERVER_BASEXYZ) != 0)
                    {
                        ESP_LOGW(TAG, "RX.RTK-1010: SETBASEXYZ %s", CONFIG_RTCM_SERVER_BASEXYZ);
                        gps_nmea_send("$PLSC,SETBASEXYZ," CONFIG_RTCM_SERVER_BASEXYZ "*");
                    }
#endif
                    gps_nmea_send("$PAIR436,1*");     // enable RTCM satellite ephemeris output
                    gps_nmea_send("$PLSC,MCBASE,1*"); // Set up the board as a RTCM caster
#endif
                }
                else if(linebuf_used > 4 && (strstr(linebuf, "$P") == &linebuf[0]))
                {
                    // RTK-1010 custom message
                    linebuf[linebuf_used - 2] = 0;
                    linebuf[linebuf_used - 1] = 0;
                    ESP_LOGW(TAG, "RX.RTK-1010: %s %02x", linebuf, gps_nmea_csum(linebuf));
                }

                if(gps_nmea_ok(linebuf))
                {
                    // https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html
                    if(strstr(linebuf, "$GNGGA") == linebuf)
                    {
                        strcpy(linebuf_bak, linebuf);
                        gps_md.uart.position_fix = gps_nmea_get_int(linebuf, 6, 0);
                        if(gps_md.ntrip.gga_print_timeout < esp_timer_get_time())
                        {
                            double lla[3] = {0};
                            double ecef[3] = {0};
                            lla[0] = ddmmtodd(gps_nmea_get_float(linebuf, 2));
                            lla[1] = ddmmtodd(gps_nmea_get_float(linebuf, 4));
                            lla[2] = gps_nmea_get_float(linebuf, 9);
                            lla2ecef(lla,ecef);

                            gps_md.ntrip.gga_print_timeout = esp_timer_get_time() + 3000000;
                            ESP_LOGW(TAG,
                                "GPS: ok: fix=%d sat=%d LLA: %.6f %.6f %.6f ECEF(m): %.6f %.6f %.6f UART: ntrip-rx=%d uart-tx=%d uart-rx=%d gps-errors=%d",
                                gps_md.uart.position_fix, gps_nmea_get_int(linebuf, 7, 0),
                                gps_nmea_get_float(linebuf, 2), gps_nmea_get_float(linebuf, 4),
                                gps_nmea_get_float(linebuf, 9), 
                                ecef[0],ecef[1],ecef[2],
                                gps_md.ntrip.ntrip_rx_bytes, gps_md.uart.tx_bytes,
                                gps_md.uart.rx_bytes, gps_md.uart.rx_errors);
                        }

                        if(gps_md.uart.position_fix >= 1)
                        {
#ifdef CONFIG_RTK1010_NODE_ROVER_NTRIP_CLIENT
                            strncpy(gps_md.ntrip.gga_line, linebuf_bak, linebuf_used);
                            gps_md.ntrip.gga_line[linebuf_used] = 0;
#endif
#ifdef CONFIG_RTK1010_NODE_ROVER_NTRIP_CLIENT
                            gps_ntrip_start();
#endif
                        }
                    }
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

        usleep(100000);
        gpio_set_level((gpio_num_t)4, 1); // release the RTK-1010 from reset
        usleep(1000000);
        ESP_LOGI(TAG, "gps_uart_start RTK-1010 EN released");

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

        uart_driver_install(UART_NUM_1, sizeof(gps_rx_buffer) * 10, 0, 0, NULL, intr_alloc_flags);
        uart_param_config(UART_NUM_1, &uart_config);
        uart_set_pin(UART_NUM_1, GPS_UART1_TXD, GPS_UART1_RXD, -1, -1);
#ifdef GPS_UART1_INV
        uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
#endif
        uart_flush(UART_NUM_1);
        gps_md.uart.initflag = 1;
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
        int len = uart_read_bytes(UART_NUM_1, gps_rx_buffer, (sizeof(gps_rx_buffer) - 2), 10 / portTICK_RATE_MS);
        if(len > 0)
        {
            if(gps_md.uart.initflag == 1)
            {
                ESP_LOGE(TAG, "GPS INIT");
                gps_md.uart.initflag = 2;
                gps_nmea_send("$PAIR004*"); // Hot Start ...
                //gps_nmea_send("$PAIR006*"); // Cold Start ...
                //gps_nmea_send("$PAIR007*"); // Try this when a base is reconfigured into a rover
            }

            gps_md.uart.rx_bytes += len;
            if(gps_md.tcpserv.client_sock != 0)
            {
                send(gps_md.tcpserv.client_sock, gps_rx_buffer, len, 0);
                // fflush(gps_md.tcpserv.client_sock);
            }
#ifdef CONFIG_RTK1010_NODE_BASE_RTCM_SERVER
            if(gps_md.rtcmtcpserv.client_sock != 0)
            {
                send(gps_md.rtcmtcpserv.client_sock, gps_rx_buffer, len, 0);
                // fflush(gps_md.tcpserv.client_sock);
            }
#endif
#ifdef CONFIG_RTK1010_NODE_PASS_TO_CDC
            tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, gps_rx_buffer, len);
            tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
#endif
            gps_rx_buffer[len] = 0;
            // ESP_LOGW(TAG, "tx %d %s", len, gps_rx_buffer);
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
    usleep(1000000);
    ESP_LOGI(TAG, "gps_task TXD=%d RXD=%d", GPS_UART1_TXD, GPS_UART1_RXD);

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
#ifdef CONFIG_RTK1010_NODE_BASE_RTCM_SERVER
        gps_rtcmtcpserv_start();
#endif
#ifdef CONFIG_RTK1010_NODE_ROVER_RTCM_CLIENT
        gps_rtcmtcpclient_start();
#endif

        while(true)
        {
            if(powerstate() == 0)
            {
                break; /* stop GPS when power down */
            }

            gps_uart_handle();
            gps_tcpserv_handle();
#ifdef CONFIG_RTK1010_NODE_BASE_RTCM_SERVER
            gps_rtcmtcpserv_handle();
#endif
#ifdef CONFIG_RTK1010_NODE_ROVER_RTCM_CLIENT
            gps_rtcmtcpclient_handle();
#endif
        }

#ifdef CONFIG_RTK1010_NODE_ROVER_NTRIP_CLIENT
        gps_ntrip_stop();
#endif
#ifdef CONFIG_RTK1010_NODE_BASE_RTCM_SERVER
        gps_rtcmtcpserv_stop();
#endif
#ifdef CONFIG_RTK1010_NODE_ROVER_RTCM_CLIENT
        gps_rtcmtcpclient_stop();
#endif
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
    gpio_num_t gpio = (gpio_num_t)4;
    gpio_hold_dis(gpio);
    gpio_set_pull_mode(gpio, GPIO_PULLDOWN_ONLY);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, 0);
    //gpio_hold_en(gpio);

#ifdef CONFIG_RTK1010_NODE_ROVER_NTRIP_CLIENT
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
        xTaskCreate(ntrip_task, "ntrip_task", 4096, NULL, 1, NULL);
    }
#endif

    xTaskCreate(gps_task, "gps_task", 4096, NULL, 2, NULL);
}
/**
 * @brief
 */
void gps_exit()
{
}

/* EOF
 */