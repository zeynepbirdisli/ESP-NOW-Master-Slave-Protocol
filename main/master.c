#include "espnow_common.h"

//----- External Log Flag -----
extern bool enable_logs;

//----- Device Configuration -----
static const char *TAG = "MASTER";
static const uint32_t MASTER_ID = 0xAAAA;
static const gpio_num_t LED_PIN = GPIO_NUM_26;
static const gpio_num_t BUTTON_PIN = GPIO_NUM_32;

//----- Identity & Pairing -----
static uint8_t self_mac[6];
static uint32_t pairing_code = 0;
static bool paired = false;
static uint32_t master_msg_counter = 0;
static uint32_t slave_device_id = 0;

//----- Elapsed Time Tracking -----
static int64_t start_time_us = 0;

//----- Flow Control & Timeout Tracking -----
static bool ack_received = true;
static TickType_t last_send_time = 0;
static TickType_t last_send_tick = 0;
static TickType_t last_ack_time = 0;

//----- Peer MAC (after pairing) -----
static uint8_t peer_mac[6] = {0};

//----- Packet Stats -----
static uint32_t packets_sent_total = 0;
static volatile bool stats_flag = false;
static esp_timer_handle_t stats_timer;

//----- Button Debounce -----
static TickType_t last_button_press = 0;
static int last_button_state = 0;

//----- FSM States -----
typedef enum {
    STATE_INIT = 0,
    STATE_PAIR_PENDING,
    STATE_PAIRED,
    STATE_SEND_DATA,
    STATE_ERROR
} master_state_t;
static master_state_t current_state = STATE_INIT;

//----- RSSI Threshold -----
#define RSSI_THRESHOLD -60
static int8_t last_rssi = 0;

//----- Stats Timer Callback -----
static void stats_timer_cb(void* arg) { stats_flag = true; }

//----- GPIO Init -----
static void init_gpio(void) {
    gpio_config_t led_cfg = {.intr_type=GPIO_INTR_DISABLE,.mode=GPIO_MODE_OUTPUT,.pin_bit_mask=(1ULL<<LED_PIN)};
    ESP_ERROR_CHECK(gpio_config(&led_cfg));
    gpio_set_level(LED_PIN,0);

    gpio_config_t btn_cfg = {.intr_type=GPIO_INTR_DISABLE,.mode=GPIO_MODE_INPUT,.pin_bit_mask=(1ULL<<BUTTON_PIN),.pull_down_en=1};
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));
}

//----- Master Initialization -----
void init_master(void) {
    init_nvs_and_netif();
    init_wifi_station();
    init_espnow();
    init_gpio();

    esp_timer_create_args_t timer_args = {.callback=stats_timer_cb,.arg=NULL,.name="stats_timer"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args,&stats_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(stats_timer,STATS_PERIOD_US));

    start_time_us = esp_timer_get_time(); 
}

//----- Peer Setup -----
void setup_peer(void) {
    add_broadcast_peer();
    esp_now_peer_info_t self_peer = {0};
    esp_wifi_get_mac(WIFI_IF_STA,self_peer.peer_addr);
    self_peer.channel = CHANNEL;
    self_peer.ifidx = WIFI_IF_STA;
    self_peer.encrypt = false;
    esp_err_t res = esp_now_add_peer(&self_peer);

    if(res == ESP_ERR_ESPNOW_EXIST){
        esp_now_mod_peer(&self_peer);
    } else if(res!=ESP_OK && enable_logs){
        ESP_LOGE(TAG,"Failed to add self as peer: %d",res);
    }
}

//----- Receive Callback -----
void master_recv_cb(const esp_now_recv_info_t *info,const uint8_t *data,int len) {
    if(!info || !data) return;

    last_rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;

    if(len==sizeof(pairing_ack_t)){
        pairing_ack_t ack;
        memcpy(&ack,data,sizeof(ack));
        if(ack.msg_id==MSG_PAIR_ACK && !paired){
            if(last_rssi>=RSSI_THRESHOLD){
                pairing_code = ack.pairing_code;
                slave_device_id = ack.device_id;

                memcpy(peer_mac, ack.mac_addr, 6);
                paired = true; 
                gpio_set_level(LED_PIN,1);
                current_state=STATE_PAIRED;

                if(enable_logs){
                    ESP_LOGI(TAG,"Pairing ACK received from Slave 0x%" PRIX32
                             " MAC=%02X:%02X:%02X:%02X:%02X:%02X, code=%" PRIu32
                             " RSSI=%d",
                             ack.device_id,
                             ack.mac_addr[0], ack.mac_addr[1], ack.mac_addr[2],
                             ack.mac_addr[3], ack.mac_addr[4], ack.mac_addr[5],
                             ack.pairing_code,last_rssi);
                }

                esp_now_peer_info_t peer_info={0};
                memcpy(peer_info.peer_addr,ack.mac_addr,6);
                peer_info.channel=CHANNEL;
                peer_info.ifidx=WIFI_IF_STA;
                peer_info.encrypt=false;
                esp_err_t res=esp_now_add_peer(&peer_info);
                if(res == ESP_ERR_ESPNOW_EXIST){
                    esp_now_del_peer(peer_info.peer_addr);
                    esp_now_add_peer(&peer_info);         
                }
                start_time_us = esp_timer_get_time(); 
            }else{
                if(enable_logs) 
                ESP_LOGW(TAG,"Pairing ignored, weak signal RSSI=%d",last_rssi);
            }
        }
    }else if(len==sizeof(data_ack_packet_t)){
        data_ack_packet_t pkt;
        memcpy(&pkt,data,sizeof(pkt));

        if (!paired || pkt.pairing_code != pairing_code || 
            memcmp(info->src_addr, peer_mac, 6) != 0) {
            esp_now_del_peer(peer_mac); 
            paired=false; gpio_set_level(LED_PIN,0);
            memset(peer_mac,0,sizeof(peer_mac));
            pairing_code=0; master_msg_counter=0; slave_device_id=0;
            ack_received=true;
            start_time_us=esp_timer_get_time();
            return;
        }

        if(enable_logs){
            ESP_LOGI(TAG,"Data ACK received from 0x%" PRIX32 
                " code=%" PRIu32 " msg_num=%" PRIu32,
                     pkt.device_id,pkt.pairing_code,pkt.msg_num);
        }

        printf("Elapsed Time: %.6f s\n", 
            (double)((esp_timer_get_time() - start_time_us) / 1000000.0));

        if(info->rx_ctrl && enable_logs){
            int rssi=info->rx_ctrl->rssi;
            int noise=info->rx_ctrl->noise_floor;
            int snr=rssi-noise;
            last_rssi=rssi;
            ESP_LOGI(TAG,"Link Quality: RSSI=%d dBm, Noise=%d dBm, SNR=%d dB → %s",
                     rssi,noise,snr,(snr>40?"Excellent":snr>25?"Good":snr>15?"Fair":"Poor"));
        }

        ack_received=true;
        master_msg_counter=(master_msg_counter%1000000)+1;
        last_ack_time=xTaskGetTickCount();
    }
}

//----- FSM Task -----
void master_task(void *arg) {
    uint8_t bcast_addr[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    data_packet_t pkt;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    for(;;){
        vTaskDelayUntil(&last_wake, period);

        switch(current_state){
            case STATE_INIT:
                
                paired=false; pairing_code=0; master_msg_counter=0; slave_device_id=0;
                ack_received=true; last_send_time=0; last_send_tick=0; last_ack_time=0;
                packets_sent_total=0; gpio_set_level(LED_PIN,0);
                memset(peer_mac, 0, sizeof(peer_mac));
                esp_now_del_peer(peer_mac); 
                start_time_us = esp_timer_get_time(); 

                current_state=STATE_PAIR_PENDING;
                break;

            case STATE_PAIR_PENDING:{
                int button_state=gpio_get_level(BUTTON_PIN);
                if(button_state==1 && last_button_state==0){
                    TickType_t now=xTaskGetTickCount();
                    if((now-last_button_press)>pdMS_TO_TICKS(DEBOUNCE_MS)){
                        last_button_press=now;
                        pairing_req_t req = {.msg_id=MSG_PAIR_REQ,.msg_num=0,
                            .device_id=MASTER_ID};
                        esp_wifi_get_mac(WIFI_IF_STA,self_mac);
                        memcpy(req.mac_addr,self_mac,6);
                        strcpy(req.device_name,"Master");
                        esp_now_send(bcast_addr,(uint8_t*)&req,sizeof(req));

                        if(enable_logs) 
                            ESP_LOGI(TAG,"Pairing request sent");
                    }
                }
                last_button_state=button_state;
                break;
            }

            case STATE_PAIRED:
                if(enable_logs) 
                    ESP_LOGI(TAG,"Device paired, switching to SEND_DATA...");
                master_msg_counter=1; ack_received=true; last_ack_time=xTaskGetTickCount();
                current_state=STATE_SEND_DATA;
                break;

            case STATE_SEND_DATA:
                if(paired){
                    if(last_rssi<RSSI_THRESHOLD){
                        if(enable_logs) 
                            ESP_LOGW(TAG,"Signal too weak, returning to INIT. RSSI=%d",last_rssi);
                        paired=false; gpio_set_level(LED_PIN,0); current_state=STATE_INIT;
                        ack_received=true;
                        break;
                    }

                    TickType_t now_ticks=xTaskGetTickCount();

                    if((now_ticks-last_ack_time)>=pdMS_TO_TICKS(PAIRING_TIMEOUT_MS)){
                        if(enable_logs) 
                            ESP_LOGW(TAG,"Long timeout, pairing failed. Resetting...");
                        paired=false; gpio_set_level(LED_PIN,0); current_state=STATE_INIT;
                        ack_received=true;
                        break;
                    }

                    if((now_ticks-last_send_tick)>=period){
                        last_send_tick=now_ticks;

                        if(!ack_received && (now_ticks-last_send_time>ACK_TIMEOUT_MS)){
                            if(enable_logs)
                                ESP_LOGW(TAG,"ACK timeout, msg_num=%" PRIu32 
                                    ", resending packet",master_msg_counter);
                            ack_received=true;
                        }

                        if(ack_received){
                            ack_received=false;
                            pkt.msg_id=MSG_DATA; pkt.msg_num=master_msg_counter; 
                            pkt.device_id=MASTER_ID; pkt.pairing_code=pairing_code;
                            for(int i=0;i<DATA_LEN;i++) pkt.data[i]=esp_random()&0xFF;

                            uint8_t *dst; // destination address
                            if (paired) {
                                dst = peer_mac; 
                            } else {
                                dst = bcast_addr;
                            }

                            esp_err_t res = esp_now_send(dst,(uint8_t*)&pkt,sizeof(pkt));
                            if(res!=ESP_OK){
                                if(enable_logs) 
                                    ESP_LOGE(TAG,"esp_now_send failed, err=%d",res);
                                ack_received=true;
                            }else{
                                last_send_time=now_ticks;
                                packets_sent_total++;
                                if(enable_logs) 
                                    ESP_LOGI(TAG,"Data packet sent to device=0x%" PRIX32  
                                        " code=%" PRIu32 " msg_num=%" PRIu32,
                                         slave_device_id,pkt.pairing_code,pkt.msg_num);
                            }
                        }
                    }
                }
                break;

            case STATE_ERROR:
                if(enable_logs) ESP_LOGE(TAG,"Error occurred!");
                break;
        }
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}