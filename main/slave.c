#include "espnow_common.h"

//----- External Log Flag -----
extern bool enable_logs;

//----- Device Configuration -----
static const char *TAG = "SLAVE";
static const uint32_t SLAVE_ID = 0xBBBB;
static const gpio_num_t LED_PIN = GPIO_NUM_27;
static const gpio_num_t BUTTON_PIN = GPIO_NUM_33;

//----- Identity & Pairing -----
static uint8_t self_mac[6];
static uint8_t master_mac[6];
static uint32_t pairing_code = 0;
static bool paired = false;
static uint32_t last_msg_num = 0;
static int64_t last_data_time = 0;
static int8_t last_rssi = 0;

//----- Elapsed Time Tracking -----
static int64_t start_time_us = 0;

//----- Timer -----
static volatile bool stats_flag = false;
static esp_timer_handle_t stats_timer;
static void stats_timer_cb(void* arg){ stats_flag=true; }

//----- Button Debounce -----
static TickType_t last_button_press = 0;
static int last_button_state = 0;

//----- FSM States -----
typedef enum {
    STATE_INIT = 0,
    STATE_PAIR_REQUESTED,
    STATE_PAIRED,
    STATE_RECEIVE_DATA,
    STATE_ERROR
} state_t;
static state_t current_state = STATE_INIT;
static uint32_t expected_msg_num = 1;

//----- GPIO Init -----
static void init_gpio(void){
    gpio_config_t led_cfg = 
    {.intr_type=GPIO_INTR_DISABLE,.mode=GPIO_MODE_OUTPUT,.pin_bit_mask=(1ULL<<LED_PIN)};
    ESP_ERROR_CHECK(gpio_config(&led_cfg));
    gpio_set_level(LED_PIN,0);

    gpio_config_t btn_cfg = {.intr_type=GPIO_INTR_DISABLE,.mode=GPIO_MODE_INPUT,
        .pin_bit_mask=(1ULL<<BUTTON_PIN),.pull_down_en=1};
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));
}

//----- Slave Initialization -----
void init_slave(void){
    init_nvs_and_netif();
    init_wifi_station();
    init_espnow();
    init_gpio();
    esp_wifi_get_mac(WIFI_IF_STA,self_mac);

    esp_timer_create_args_t timer_args=
    {.callback=stats_timer_cb,.arg=NULL,.name="stats_timer"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args,&stats_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(stats_timer,STATS_PERIOD_US));

    start_time_us = esp_timer_get_time();  
}

//----- Receive Callback -----
void slave_recv_cb(const esp_now_recv_info_t *info,const uint8_t *data,int len){
    if(!info || !data) return;
    last_data_time=esp_timer_get_time();

    if(len==sizeof(pairing_req_t)){
        pairing_req_t req; memcpy(&req,data,sizeof(req));
        if(req.msg_id==MSG_PAIR_REQ){
            memcpy(master_mac,info->src_addr,6);
            current_state=STATE_PAIR_REQUESTED;

            if(enable_logs) ESP_LOGI(TAG,"Pairing request received from 0x%" PRIX32
                 " name=%s MAC=%02X:%02X:%02X:%02X:%02X:%02X",
                     req.device_id,req.device_name,
                     info->src_addr[0],info->src_addr[1],info->src_addr[2],
                     info->src_addr[3],info->src_addr[4],info->src_addr[5]);
        }
    } else if(len==sizeof(data_packet_t)){
        data_packet_t pkt; memcpy(&pkt,data,sizeof(pkt));

        if(enable_logs){
            printf("\n");
            ESP_LOGI(TAG,"Data packet received from 0x%" PRIX32
                 " msg_num=%" PRIu32 " code=%" PRIu32 " RSSI=%d",
                     pkt.device_id,pkt.msg_num,pkt.pairing_code,last_rssi);
        }
        printf("Elapsed time after data: %.6f s\n",
            (double)((esp_timer_get_time()-start_time_us) / 1000000.0));

        if (memcmp(info->src_addr, master_mac, 6) != 0) {
            if(enable_logs) ESP_LOGW(TAG, "Ignored DATA: src MAC != master_mac");
        } else if (paired && pkt.pairing_code == pairing_code) {
            data_ack_packet_t ack_pkt= {.msg_id=MSG_DATA_ACK,.msg_num=pkt.msg_num,
                .device_id=SLAVE_ID,.pairing_code=pairing_code};
            esp_err_t res=esp_now_send(master_mac,(uint8_t*)&ack_pkt,sizeof(ack_pkt));
            if(res==ESP_OK){
                if(enable_logs)
                    ESP_LOGI(TAG,"ACK sent to 0x%" PRIX32 
                        " code=%" PRIu32 " msg_num=%" PRIu32, 
                        pkt.device_id,ack_pkt.pairing_code,ack_pkt.msg_num);
            } else if(enable_logs){
                ESP_LOGE(TAG,"esp_now_send ACK failed, err=%d",res);
            }
        } else {
            if(enable_logs) 
                ESP_LOGW(TAG, "Ignored DATA: not paired or pairing_code mismatch (got=%"
                     PRIu32 " expected=%" PRIu32 ")",
                     pkt.pairing_code, pairing_code);
        }

        current_state=STATE_RECEIVE_DATA;
    }
}

//----- FSM Task ----- 
void slave_task(void *arg){
    const int64_t TIMEOUT_US=100000;
    for(;;){
        int64_t now=esp_timer_get_time();
        int button_state=gpio_get_level(BUTTON_PIN);
        TickType_t tick_now=xTaskGetTickCount();

        if(button_state && (tick_now-last_button_press)>pdMS_TO_TICKS(DEBOUNCE_MS) && last_button_state==0){
            last_button_press=tick_now;
            last_button_state=1;
        }else if(!button_state) last_button_state=0;

        if(paired && (now-last_data_time>TIMEOUT_US)){
            if(enable_logs) ESP_LOGW(TAG,"Connection lost! Resetting state...");
            esp_now_del_peer(master_mac);
            memset(master_mac,0,sizeof(master_mac));
            current_state=STATE_INIT;

            start_time_us = esp_timer_get_time();

            if(enable_logs) ESP_LOGI(TAG,"Entered STATE_INIT");
        }

        switch(current_state){
            case STATE_INIT:
                paired=false; pairing_code=0; last_msg_num=0; expected_msg_num=1;
                memset(master_mac,0,sizeof(master_mac));
                gpio_set_level(LED_PIN,0);
                vTaskDelay(pdMS_TO_TICKS(10));
                break;

            case STATE_PAIR_REQUESTED:
                if(!paired && last_button_state){
                    last_button_state=0;
                    pairing_ack_t ack={.msg_id=MSG_PAIR_ACK,.msg_num=0,.device_id=SLAVE_ID,.pairing_code=esp_random()};
                    esp_wifi_get_mac(WIFI_IF_STA,self_mac);
                    memcpy(ack.mac_addr,self_mac,6);
                    strcpy(ack.device_name,"Slave");

                    esp_now_peer_info_t peer = {0};  
                    memcpy(peer.peer_addr, master_mac, 6);
                    peer.channel = CHANNEL;
                    peer.ifidx = WIFI_IF_STA;
                    peer.encrypt = false;

                    esp_now_del_peer(master_mac);
                    esp_now_add_peer(&peer);

                    esp_now_send(master_mac,(uint8_t*)&ack,sizeof(ack));
                    pairing_code = ack.pairing_code;
                    paired = true;
                    gpio_set_level(LED_PIN,1);
                    current_state = STATE_PAIRED;
                    last_data_time = esp_timer_get_time();
                    start_time_us = last_data_time; 
                    if(enable_logs) ESP_LOGI(TAG,"Pairing ACK sent to master, code=%" PRIu32, pairing_code);
                }
                break;

            case STATE_PAIRED:
                if(enable_logs) ESP_LOGI(TAG,"Device paired, ready to receive data...");
                last_msg_num=0;
                current_state=STATE_RECEIVE_DATA;
                break;

            case STATE_RECEIVE_DATA: break;

            case STATE_ERROR:
                if(enable_logs) ESP_LOGE(TAG,"Error state reached on SLAVE!");
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}