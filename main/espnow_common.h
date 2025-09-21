#ifndef ESPNOW_COMMON_H
#define ESPNOW_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_random.h"

//----- Common Constants -----
#define CHANNEL             6
#define DATA_LEN            200
#define MAX_NAME_LEN        16

//----- Timeouts -----
extern TickType_t ACK_TIMEOUT_MS;     
#define PAIRING_TIMEOUT_MS  1000
#define DEBOUNCE_MS         50
#define STATS_PERIOD_US     1000000ULL

//----- Message IDs -----
typedef enum {
    MSG_PAIR_REQ = 1,
    MSG_PAIR_ACK = 2,
    MSG_DATA     = 3,
    MSG_DATA_ACK = 4
} msg_id_t;

//----- Packet Structs -----
typedef struct __attribute__((packed)) {
    uint32_t msg_id;
    uint32_t msg_num;
    uint32_t device_id;
    uint8_t  mac_addr[6];
    char     device_name[MAX_NAME_LEN];
} pairing_req_t;

typedef struct __attribute__((packed)) {
    uint32_t msg_id;
    uint32_t msg_num;
    uint32_t device_id;
    uint8_t  mac_addr[6];
    char     device_name[MAX_NAME_LEN];
    uint32_t pairing_code;
} pairing_ack_t;

typedef struct __attribute__((packed)) {
    uint32_t msg_id;
    uint32_t msg_num;
    uint32_t device_id;
    uint32_t pairing_code;
    uint8_t  data[DATA_LEN];
} data_packet_t;

typedef struct __attribute__((packed)) {
    uint32_t msg_id;
    uint32_t msg_num;
    uint32_t device_id;
    uint32_t pairing_code;
} data_ack_packet_t;

//----- Add Broadcast Peer -----
static inline void add_broadcast_peer(void)
{
    esp_now_peer_info_t peer = {0};
    uint8_t bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    memcpy(peer.peer_addr, bcast, 6);
    peer.channel = CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t r = esp_now_add_peer(&peer);
    if (r != ESP_OK && r != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW("ESPNOW", "Add Broadcast Peer Failed: %d", r);
    }
}

//----- Init NVS And Netif -----
static inline esp_err_t init_nvs_and_netif(void)
{
    esp_err_t r = nvs_flash_init();
    if (r != ESP_OK && r != ESP_ERR_NVS_NO_FREE_PAGES && r != ESP_ERR_NVS_NEW_VERSION_FOUND) {
        return r;
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    return ESP_OK;
}

//----- Init WiFi Station -----
static inline void init_wifi_station(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
}

//----- Init ESP-NOW -----
static inline esp_err_t init_espnow(void)
{
    return esp_now_init();
}

#endif /* ESPNOW_COMMON_H */