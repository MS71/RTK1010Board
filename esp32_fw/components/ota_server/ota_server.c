//#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include <string.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "esp_wifi.h"

#include "ota_server.h"

static const char * TAG = "OTA";
//static char ota_buff[OTA_BUFF_SIZE] = {};

/*socket*/
static int connect_socket = 0;

static int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
    int err = getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen);
    if (err == -1) {
        ESP_LOGE(TAG, "getsockopt failed:%s", strerror(err));
        return -1;
    }
    return result;
}

static int show_socket_error_reason(const char *str, int socket)
{
    int err = get_socket_error_code(socket);

    if (err != 0) {
        ESP_LOGW(TAG, "%s socket error %d %s", str, err, strerror(err));
    }

    return err;
}

int ota_log(const char *format, va_list args)
{
  return 0;
}

static esp_err_t create_tcp_server()
{
    ESP_LOGI(TAG, "server socket....port=%d", OTA_LISTEN_PORT);
    int server_socket = 0;
    struct sockaddr_in server_addr;
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) {
        show_socket_error_reason("create_server", server_socket);
        return ESP_FAIL;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(OTA_LISTEN_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        show_socket_error_reason("bind_server", server_socket);
        close(server_socket);
        return ESP_FAIL;
    }
    if (listen(server_socket, 5) < 0) {
        show_socket_error_reason("listen_server", server_socket);
        close(server_socket);
        return ESP_FAIL;
    }
    struct sockaddr_in client_addr;
    unsigned int socklen = sizeof(client_addr);
    connect_socket = accept(server_socket, (struct sockaddr *)&client_addr, &socklen);
    if (connect_socket < 0) {
        show_socket_error_reason("accept_server", connect_socket);
        close(server_socket);
        return ESP_FAIL;
    }
    /*connection established，now can send/recv*/
    ESP_LOGI(TAG, "tcp connection established!");
    return ESP_OK;
}

void ota_server_start()
{
    ESP_ERROR_CHECK( create_tcp_server() );

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
    	update_partition->subtype, update_partition->address);

    int recv_len;
    bool is_req_body_started = false;
    int content_length = -1;
    int content_received = 0;
	int n=0;
    char* ota_buff = malloc(OTA_BUFF_SIZE);
    if( ota_buff == NULL )
    {
        ESP_LOGE(TAG, "can't allocate %d bytes from HEAP",OTA_BUFF_SIZE);
        return;
    }

#ifdef CONFIG_PM_ENABLE
    {
        wifi_config_t wifi_config = {};
        if( ESP_OK == esp_wifi_get_config(WIFI_IF_STA, &wifi_config))
        {
            ESP_LOGI(TAG, "wifi_config.listen_interval = 1");
            wifi_config.sta.listen_interval = 1;
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        }
    }
#endif

    esp_ota_handle_t ota_handle;
    do {
        recv_len = recv(connect_socket, ota_buff, OTA_BUFF_SIZE, 0);
        if (recv_len > 0) {
            if (!is_req_body_started) {
                const char *content_length_start = "Content-Length: ";
                char *content_length_start_p = strstr(ota_buff, content_length_start) + strlen(content_length_start);
                sscanf(content_length_start_p, "%d", &content_length);
                ESP_LOGW(TAG, "Detected content length: %d", content_length);
                ESP_ERROR_CHECK( esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle) );
                const char *header_end = "\r\n\r\n";
                char *body_start_p = strstr(ota_buff, header_end) + strlen(header_end);
                int body_part_len = recv_len - (body_start_p - ota_buff);
                esp_ota_write(ota_handle, body_start_p, body_part_len);
                content_received += body_part_len;
                is_req_body_started = true;
            } else {
                esp_ota_write(ota_handle, ota_buff, recv_len);
                content_received += recv_len;
				if( ((n++) & 63) == 0 )
				{
					ESP_LOGW(TAG, "... %d ...", content_received);
				}
            }
        }
        else if (recv_len < 0) {
            ESP_LOGE(TAG, "Error: recv data error! errno=%d", errno);
        }
    } while (recv_len > 0 && content_received < content_length);

    ESP_LOGW(TAG, "Binary transferred finished: %d bytes", content_received);

    ESP_ERROR_CHECK( esp_ota_end(ota_handle) );
    esp_err_t err = esp_ota_set_boot_partition(update_partition);
    char res_buff[128];
    int send_len;
    if (err == ESP_OK) {
        send_len = sprintf(res_buff, "200 OK\n\nSuccess. Next boot partition is %s\n", update_partition->label);
    } else {
        send_len = sprintf(res_buff, "400 Bad Request\n\nFailure. Error code: 0x%x\n", err);
    }
    send(connect_socket, res_buff, send_len, 0);
    close(connect_socket);

    const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
    ESP_LOGW(TAG, "Next boot partition subtype %d at offset 0x%x",
    	boot_partition->subtype, boot_partition->address);
    ESP_LOGW(TAG, "Prepare to restart system!");
    esp_log_set_vprintf(ota_log);
    usleep(1000000);
    esp_restart();
}
