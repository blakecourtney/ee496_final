// GROUND STATION FIRMWARE
#include <string.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "config.h"
#include "driver/gpio.h"

//Constants
#define RX_SIZE          (1500)

//Serial to Python GUI
#define GUI_UART         UART_NUM_0
#define GUI_TX_PIN       1
#define GUI_RX_PIN       3
#define GUI_BAUD         115200

//Rx/Tx LEDs
#define LED_RX_PIN   25
#define LED_TX_PIN   26
#define LED_BLINK_MS 50
static QueueHandle_t led_queue = NULL;
#define LED_BLINK_RX 0
#define LED_BLINK_TX 1

#define BLINK_RX() do { uint8_t _b = LED_BLINK_RX; xQueueSend(led_queue, &_b, 0); } while(0)
#define BLINK_TX() do { uint8_t _b = LED_BLINK_TX; xQueueSend(led_queue, &_b, 0); } while(0)

//Variable Definitions
static const char *MESH_TAG = "mesh_main";
static const uint8_t s_mesh_id[] = MESH_ID;
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_mesh_connected = false;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;

//LED functions
void led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_RX_PIN) | (1ULL << LED_TX_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_RX_PIN, 0);
    gpio_set_level(LED_TX_PIN, 0);
    ESP_LOGI(MESH_TAG, "LEDs initialized");
}

void led_task(void *arg)
{
    uint8_t which;
    while (1) {
        if (xQueueReceive(led_queue, &which, portMAX_DELAY) == pdTRUE) {
            int pin = (which == LED_BLINK_RX) ? LED_RX_PIN : LED_TX_PIN;
            gpio_set_level(pin, 1);
            vTaskDelay(pdMS_TO_TICKS(LED_BLINK_MS));
            gpio_set_level(pin, 0);
        }
    }
    vTaskDelete(NULL);
}

//Serial to GUI
void gui_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = GUI_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(GUI_UART, &cfg);
    uart_set_pin(GUI_UART, GUI_TX_PIN, GUI_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(GUI_UART, 2048, 0, 0, NULL, 0);
    ESP_LOGI(MESH_TAG, "GUI UART initialized");
}

void send_to_gui(uint8_t *data, size_t len)
{
    uart_write_bytes(GUI_UART, data, len);
    uart_write_bytes(GUI_UART, "\n", 1);
}

void gui_rx_task(void *arg)
{
    uint8_t buf[sizeof(packet_t)];
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    uint8_t sta_mac[6];
    uint8_t ap_mac[6];

    while (1) {
        //read exactly one packet from GUI
        int len = uart_read_bytes(GUI_UART, buf, sizeof(packet_t), 100 / portTICK_PERIOD_MS);
        if (len < sizeof(packet_t)) continue;

        packet_t *pkt = (packet_t *)buf;
        if (pkt->start != PKT_START || pkt->end != PKT_END) continue;

        ESP_LOGI(MESH_TAG, "[GND] received cmd type:%d for drone:%d", pkt->type, pkt->drone_id);

        //get routing table
        esp_wifi_get_mac(WIFI_IF_STA, sta_mac);
        esp_wifi_get_mac(WIFI_IF_AP, ap_mac);
        esp_mesh_get_routing_table((mesh_addr_t *)&route_table,
                                   CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

        //forward to target drone or broadcast 
        for (int i = 0; i < route_table_size; i++) {
            if (memcmp(route_table[i].addr, sta_mac, 6) == 0) continue;
            if (memcmp(route_table[i].addr, ap_mac, 6) == 0) continue;

            // if drone_id is 0 send to all, otherwise match last byte of MAC
            if (pkt->drone_id != 0 && route_table[i].addr[5] != pkt->drone_id) continue;

            mesh_data_t data = {
                .data  = buf,
                .size  = sizeof(packet_t),
                .proto = MESH_PROTO_BIN,
                .tos   = MESH_TOS_P2P,
            };
            esp_err_t err = esp_mesh_send(&route_table[i], &data, MESH_DATA_P2P, NULL, 0);
            ESP_LOGI(MESH_TAG, "[GND-TX] forwarded to drone:%d err:0x%x",
                     route_table[i].addr[5], err);
            BLINK_TX();
        }
    }
    vTaskDelete(NULL);
}

//Mesh TX Task
//checks if this esp is the root node (it should be)
//every 5 seconds check how many drones are in the mesh
//MAC address is used for routing table entries
    //skips own MAC address to have accurate count of drones in air not nodes in network
void esp_mesh_p2p_tx_main(void *arg)
{
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    uint8_t sta_mac[6];
    uint8_t ap_mac[6];

    while (1) {
        if (!esp_mesh_is_root()) {
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        esp_wifi_get_mac(WIFI_IF_STA, sta_mac);
        esp_wifi_get_mac(WIFI_IF_AP, ap_mac);
        esp_mesh_get_routing_table((mesh_addr_t *)&route_table,
                                   CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

        int drone_count = 0;
        for (int i = 0; i < route_table_size; i++) {
            if (memcmp(route_table[i].addr, sta_mac, 6) == 0) continue;
            if (memcmp(route_table[i].addr, ap_mac, 6) == 0) continue;
            drone_count++;
        }

        if (drone_count > 0) {
            ESP_LOGI(MESH_TAG, "[GND-TX] %d drone(s) in mesh", drone_count);
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

//Helper: send ACK to drone
//haven't really used this yet
//to use with photo transmission
void send_ack(mesh_addr_t *dest, uint8_t ack_type, uint16_t chunk_index)
{
    packet_t pkt = {
        .start    = PKT_START,
        .drone_id = 0,
        .type     = PKT_TYPE_ACK,
        .end      = PKT_END,
    };
    ack_t ack = {
        .ack_type    = ack_type,
        .chunk_index = chunk_index,
    };
    memcpy(pkt.payload, &ack, sizeof(ack_t));

    mesh_data_t data = {
        .data  = (uint8_t *)&pkt,
        .size  = sizeof(pkt),
        .proto = MESH_PROTO_BIN,
        .tos   = MESH_TOS_P2P,
    };
    esp_mesh_send(dest, &data, MESH_DATA_P2P, NULL, 0);
}

//send flag ACK (request photo)
void send_flag_ack(mesh_addr_t *dest)
{
    packet_t pkt = {
        .start    = PKT_START,
        .drone_id = 0,
        .type     = PKT_TYPE_FLAG_ACK,
        .end      = PKT_END,
    };
    memset(pkt.payload, 0, sizeof(pkt.payload));

    mesh_data_t data = {
        .data  = (uint8_t *)&pkt,
        .size  = sizeof(pkt),
        .proto = MESH_PROTO_BIN,
        .tos   = MESH_TOS_P2P,
    };
    esp_mesh_send(dest, &data, MESH_DATA_P2P, NULL, 0);
    ESP_LOGI(MESH_TAG, "[GND-TX] flag ACK sent - photo requested");
}

//Mesh RX Task
//called on any received data from the mesh
//primarily forwards to GUI in this case 
//GND receives telemetry/flags/photos from drones
//modified from ESP-IDF
void esp_mesh_p2p_rx_main(void *arg)
{
    esp_err_t err;
    mesh_addr_t from;
    mesh_data_t data;
    int flag = 0;
    data.data = rx_buf;
    data.size = RX_SIZE;

    while (1) {
        data.size = RX_SIZE;
        err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (err != ESP_OK || !data.size) {
            ESP_LOGE(MESH_TAG, "err:0x%x, size:%d", err, data.size);
            continue;
        }
        BLINK_RX();

        // DEBUG
        ESP_LOGI(MESH_TAG, "[GND-RX] Caught a packet! Size: %d bytes (Expected: %d)", data.size, sizeof(packet_t));

        // handle photo chunks (larger packet type)
        if (data.size >= sizeof(photo_packet_t)) {
            photo_packet_t *photo = (photo_packet_t *)data.data;
            if (photo->start == PKT_START && photo->end == PKT_END &&
                photo->type == PKT_TYPE_PHOTO_CHUNK) {
                ESP_LOGI(MESH_TAG, "[GND-RX] photo chunk %d/%d drone:%d len:%d",
                         photo->chunk_index, photo->total_chunks,
                         photo->drone_id, photo->data_len);

                // forward chunk to GUI
                send_to_gui((uint8_t *)photo, sizeof(photo_packet_t));

                // ACK the chunk
                send_ack(&from, PKT_TYPE_PHOTO_CHUNK, photo->chunk_index);
                continue;
            }
        }

        // handle standard packets
        if (data.size >= sizeof(packet_t)) {
            packet_t *pkt = (packet_t *)data.data;
            if (pkt->start == PKT_START && pkt->end == PKT_END) {
                switch (pkt->type) {

                    case PKT_TYPE_TELEMETRY: {
                        telemetry_t *tlm = (telemetry_t *)pkt->payload;
                        ESP_LOGI(MESH_TAG,
                                 "[GND-RX] telemetry drone:%d lat:%.5f lon:%.5f alt:%.1f bat:%.2fV armed:%d",
                                 pkt->drone_id, tlm->lat, tlm->lon, tlm->alt,
                                 tlm->battery, tlm->armed);
                        // forward to Python GUI
                        send_to_gui((uint8_t *)pkt, sizeof(packet_t));
                        break;
                    }

                    case PKT_TYPE_FLAG: {
                        flag_t *f = (flag_t *)pkt->payload;
                        ESP_LOGW(MESH_TAG,
                                 "[GND-RX] PERSON DETECTED drone:%d conf:%.2f lat:%.5f lon:%.5f alt:%.1f",
                                 pkt->drone_id, f->confidence, f->lat, f->lon, f->alt);
                        // forward alert to GUI
                        send_to_gui((uint8_t *)pkt, sizeof(packet_t));
                        // request photo from drone
                        send_flag_ack(&from);
                        break;
                    }

                    case PKT_TYPE_HEARTBEAT: {
                        ESP_LOGI(MESH_TAG, "[GND-RX] heartbeat drone:%d", pkt->drone_id);
                        send_to_gui((uint8_t *)pkt, sizeof(packet_t));
                        break;
                    }

                    case PKT_TYPE_PHOTO_DONE: {
                        ESP_LOGI(MESH_TAG, "[GND-RX] photo transfer complete drone:%d", pkt->drone_id);
                        send_to_gui((uint8_t *)pkt, sizeof(packet_t));
                        break;
                    }

                    case PKT_TYPE_PHOTO_START: {
                        ESP_LOGI(MESH_TAG, "[GND-RX] photo transfer started drone:%d", pkt->drone_id);
                            send_to_gui((uint8_t *)pkt, sizeof(packet_t));
                        break;
                    }

                    default:
                        ESP_LOGW(MESH_TAG, "[GND-RX] unknown type:%d drone:%d",
                                 pkt->type, pkt->drone_id);
                        break;
                }
            }
        }
    }
    vTaskDelete(NULL);
}

//starts the three "mesh tasks"
    //esp_mesh_p2p_tx
    //esp_mesh_p2p_rx
    //gui_rx_task
esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
        xTaskCreate(gui_rx_task, "GUIRX", 3072, NULL, 5, NULL);
    }
    return ESP_OK;
}

//Mesh Event Handler
//modified from ESP-IDF
void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint16_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_comm_p2p_start();  // always start, not just when root
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
        case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                child_connected->aid, MAC2STR(child_connected->mac));
        esp_mesh_comm_p2p_start();  // start comms when first drone connects
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid, MAC2STR(child_disconnected->mac));
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d, layer:%d",
                 routing_table->rt_size_change, routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d, layer:%d",
                 routing_table->rt_size_change, routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: { //logs how many attempts looking for parent
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d", no_parent->scan_times);
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d",
                 last_layer, mesh_layer);
        last_layer = mesh_layer;
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_SCAN_DONE>number:%d", scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d", network_state->is_rootless);
    }
    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, ID:"MACSTR"",
                last_layer, mesh_layer, MAC2STR(id.addr));
        last_layer = mesh_layer;
        is_mesh_connected = true;
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
            esp_netif_dhcpc_start(netif_sta);
        }
        esp_mesh_comm_p2p_start();
    }
    break;
    default:
        ESP_LOGI(MESH_TAG, "unknown event id:%" PRId32 "", event_id);
        break;
    }
}

//app_main
//modified from ESP-IDF
void app_main(void)
{
    //system init
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));

    //wifi init
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());

    //mesh init
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mesh_set_topology(CONFIG_MESH_TOPOLOGY));
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));

    //led init
    led_queue = xQueueCreate(10, sizeof(uint8_t));
    led_init();
    xTaskCreate(led_task, "LED", 1024, NULL, 3, NULL);

    //mesh config — channel 6 anchored to GND_ANCHOR ESP32
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    memcpy((uint8_t *) &cfg.mesh_id, s_mesh_id, 6);
    cfg.channel = 6;
    char anchor_ssid[] = "GND_ANCHOR";
    cfg.router.ssid_len = strlen(anchor_ssid);
    memcpy((uint8_t *) &cfg.router.ssid, anchor_ssid, cfg.router.ssid_len);
    memcpy((uint8_t *) &cfg.router.password, "gndanchor", strlen("gndanchor"));

    //force ground station to always be root
    ESP_ERROR_CHECK(esp_mesh_set_capacity_num(1000));
    ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, false));

    //mesh AP config — drones connect to this
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD, strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

    //start GUI serial and mesh
    gui_uart_init();
    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGW(MESH_TAG, "GROUND STATION (ROOT) STARTED heap:%" PRId32,
             esp_get_minimum_free_heap_size());
}