#include "espnow_common.h"

//----- Device Type -----
#define DEVICE_MASTER 1
#define DEVICE_SLAVE  2
#define DEVICE_TYPE   DEVICE_MASTER // Choose the device type

#if (DEVICE_TYPE != DEVICE_MASTER) && (DEVICE_TYPE != DEVICE_SLAVE)
#error "DEVICE_TYPE Must Be DEVICE_MASTER Or DEVICE_SLAVE"
#endif

//----- Log Control -----
bool enable_logs = false;    // true = logs on, false = logs off
TickType_t ACK_TIMEOUT_MS;     

//----- Function Prototypes -----
void init_master(void);
void init_slave(void);
void setup_peer(void);

void master_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void master_task(void *arg);

void slave_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void slave_task(void *arg);

//----- Main App -----
void app_main(void)
{
    // Set ACK timeout based on logging
    ACK_TIMEOUT_MS = pdMS_TO_TICKS(enable_logs ? 50 : 10);

    if (DEVICE_TYPE == DEVICE_MASTER) {
        init_master();
        setup_peer();
        esp_now_register_recv_cb(master_recv_cb);
        xTaskCreatePinnedToCore(master_task, "MasterTask", 4096, NULL, 8, NULL, tskNO_AFFINITY);
    }

    if (DEVICE_TYPE == DEVICE_SLAVE) {
        init_slave();
        esp_now_register_recv_cb(slave_recv_cb);
        xTaskCreatePinnedToCore(slave_task, "SlaveTask", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    }
}
