#ifndef _OTA_SERVER_H_
#define _OTA_SERVER_H__

#define OTA_LISTEN_PORT 8032
#define OTA_BUFF_SIZE 1024

#if defined(__cplusplus)  
extern "C" {
#endif

void ota_server_start();

#if defined(__cplusplus)  
}
#endif

#endif /* _OTA_SERVER_H_ */
