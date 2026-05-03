//DRONE FIRMWARE
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "common/mavlink.h"
#include "config.h"

//constants
#define RX_SIZE          (1500)

//mavlink uart (Pixhawk)
#define MAV_UART         UART_NUM_0 
#define MAV_TX_PIN       1
#define MAV_RX_PIN       3
#define MAV_BAUD         115200

//esp cam uart
#define ML_UART          UART_NUM_1
#define ML_TX_PIN        17
#define ML_RX_PIN        16
#define ML_BAUD          115200

//definitions
static const char *MESH_TAG = "mesh_main";
static const uint8_t s_mesh_id[] = MESH_ID;
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_mesh_connected = false;
static int mesh_layer = -1;
static mesh_addr_t root_addr;
static bool root_addr_known = false;
static esp_netif_t *netif_sta = NULL;

//mavlink telemetry
    //mavlink task updates this
    //read by tx task
static telemetry_t latest_telemetry = {0};
static bool telemetry_valid = false;

//Image buffer 
#define IMAGE_BUF_SIZE (115200)  // 240 * 240 * 2 (RGB565)
static uint8_t *image_buf = NULL;

//mavlink uart functions
void mavlink_init(void)
{
    uart_config_t cfg = {
        .baud_rate = MAV_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(MAV_UART, &cfg);
    uart_set_pin(MAV_UART, MAV_TX_PIN, MAV_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(MAV_UART, 2048, 0, 0, NULL, 0);
    ESP_LOGI(MESH_TAG, "MAVLink UART initialized");
}

void send_mavlink_waypoint(waypoint_t *wp)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_mission_item_int_pack(
        1, 1, &msg,
        1, 1,                               // target system/component
        0,                                  // sequence
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MAV_CMD_NAV_WAYPOINT,
        1,                                  // current
        1,                                  // autocontinue
        0, 0, 0, 0,                         // params 1-4
        (int32_t)(wp->lat * 1e7),
        (int32_t)(wp->lon * 1e7),
        wp->alt,
        MAV_MISSION_TYPE_MISSION            // mission_type - missing arg
    );
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(MAV_UART, buf, len);
    ESP_LOGI(MESH_TAG, "[MAV-TX] waypoint lat:%.5f lon:%.5f alt:%.1f",
            wp->lat, wp->lon, wp->alt);
}

void send_mavlink_command(uint8_t command_type)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    float param1 = (command_type == 1) ? 1.0f : 0.0f; // 1=arm, 0=disarm

    mavlink_msg_command_long_pack(
        1, 1, &msg,
        1, 1,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        param1, 0, 0, 0, 0, 0, 0
    );

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    uart_write_bytes(MAV_UART, buf, len);
    ESP_LOGI(MESH_TAG, "[MAV-TX] command type:%d", command_type);
}

void mavlink_task(void *arg)
{
    uint8_t byte;
    mavlink_message_t msg;
    mavlink_status_t status;

    //delay on startup
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1) {
        int len = uart_read_bytes(MAV_UART, &byte, 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
                switch (msg.msgid) {

                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
                        mavlink_global_position_int_t pos;
                        mavlink_msg_global_position_int_decode(&msg, &pos);
                        latest_telemetry.lat = pos.lat / 1e7f;
                        latest_telemetry.lon = pos.lon / 1e7f;
                        latest_telemetry.alt = pos.alt / 1000.0f;
                        telemetry_valid = true;
                        break;
                    }

                    case MAVLINK_MSG_ID_ATTITUDE: {
                        mavlink_attitude_t att;
                        mavlink_msg_attitude_decode(&msg, &att);
                        latest_telemetry.roll  = att.roll;
                        latest_telemetry.pitch = att.pitch;
                        latest_telemetry.yaw   = att.yaw;
                        break;
                    }

                    case MAVLINK_MSG_ID_SYS_STATUS: {
                        mavlink_sys_status_t sys;
                        mavlink_msg_sys_status_decode(&msg, &sys);
                        latest_telemetry.battery = sys.voltage_battery / 1000.0f;
                        break;
                    }

                    case MAVLINK_MSG_ID_GPS_RAW_INT: {
                        mavlink_gps_raw_int_t gps;
                        mavlink_msg_gps_raw_int_decode(&msg, &gps);
                        latest_telemetry.satellites = gps.satellites_visible;
                        break;
                    }

                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        latest_telemetry.armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? 1 : 0;
                        ESP_LOGI(MESH_TAG, "[MAV-RX] heartbeat armed:%d", latest_telemetry.armed);
                        break;
                    }
                }
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

//esp cam uart and functions
void ml_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = ML_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(ML_UART, &cfg);
    uart_set_pin(ML_UART, ML_TX_PIN, ML_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(ML_UART, 4096, 0, 0, NULL, 0);
    ESP_LOGI(MESH_TAG, "ML UART initialized");
}

typedef enum {
    ML_STATE_IDLE,
    ML_STATE_WAITING_START,
    ML_STATE_RECEIVING,
} ml_state_t;

static ml_state_t ml_state = ML_STATE_IDLE;

void ml_uart_task(void *arg)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint8_t drone_id = mac[5];

    uint8_t buf[512];
    size_t image_len = 0;
    uint16_t chunk_index = 0;
    uint16_t total_chunks = 0;
    bool flag_not_sent = true;

    while (1) {
        int len = uart_read_bytes(ML_UART, buf, sizeof(buf) - 1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;
        buf[len] = '\0';

        if (ml_state == ML_STATE_IDLE) {
            // wait for human detected
            if (strncmp((char *)buf, "human detected!\r\n", 17) == 0) {
                ESP_LOGW(MESH_TAG, "[ML-RX] human detected");
                uart_write_bytes(ML_UART, "ACK\n", 4);
                ml_state = ML_STATE_WAITING_START;
                flag_not_sent = true;
            }
        }
        else if (ml_state == ML_STATE_WAITING_START) {
            //wait for start image
            // forward flag to ground
            if(flag_not_sent){
                flag_not_sent = false;
                if (root_addr_known) {
                    packet_t pkt = {
                        .start    = PKT_START,
                        .drone_id = drone_id,
                        .type     = PKT_TYPE_FLAG,
                        .end      = PKT_END,
                    };
                    flag_t flag_payload = {
                        .lat = latest_telemetry.lat,
                        .lon = latest_telemetry.lon,
                        .alt = latest_telemetry.alt,
                        .confidence = 1.0f,
                    };
                    memcpy(pkt.payload, &flag_payload, sizeof(flag_t));
                    mesh_data_t data = {
                        .data = (uint8_t *)&pkt,
                        .size = sizeof(pkt),
                        .proto = MESH_PROTO_BIN,
                        .tos = MESH_TOS_P2P,
                    };
                    esp_mesh_send(&root_addr, &data, MESH_DATA_P2P, NULL, 0);
                }
            }
            if (strncmp((char *)buf, "START_IMAGE\r\n", 13) == 0) {
                ESP_LOGI(MESH_TAG, "[ML-RX] image transfer started");
                image_len = 0;
                ml_state = ML_STATE_RECEIVING;
                
                // capture any image bytes after START_IMAGE
                size_t extra = len - 13;
                if (extra > 0 && extra <= IMAGE_BUF_SIZE) {
                    memcpy(image_buf, buf + 13, extra);
                    image_len = extra;
                }
            }
        }
        else if (ml_state == ML_STATE_RECEIVING) {
            uint8_t *end_marker = (uint8_t *)memmem(buf, len, "END_IMAGE", 9);
            if (end_marker != NULL) {
                size_t bytes_before = end_marker - buf;
                if (bytes_before > 0 && image_len + bytes_before <= IMAGE_BUF_SIZE) {
                    memcpy(image_buf + image_len, buf, bytes_before);
                    image_len += bytes_before;
                }

                uart_write_bytes(ML_UART, "CLEAR\n", 6);
                ESP_LOGI(MESH_TAG, "[ML-TX] CLEAR sent");
                ml_state = ML_STATE_IDLE;
                ESP_LOGI(MESH_TAG, "[ML-RX] image complete %d bytes", image_len);

                // send chunks over mesh
                if (root_addr_known && image_len > 0) {

                    //send photo start
                    packet_t start_pkt = {
                        .start    = PKT_START,
                        .drone_id = drone_id,
                        .type     = PKT_TYPE_PHOTO_START,
                        .end      = PKT_END,
                    };
                    memset(start_pkt.payload, 0, sizeof(start_pkt.payload));
                    mesh_data_t start_data = {
                        .data  = (uint8_t *)&start_pkt,
                        .size  = sizeof(start_pkt),
                        .proto = MESH_PROTO_BIN,
                        .tos   = MESH_TOS_P2P,
                    };
                    esp_mesh_send(&root_addr, &start_data, MESH_DATA_P2P, NULL, 0);
                    ESP_LOGI(MESH_TAG, "[ML-TX] photo START sent");

                    uint16_t chunk_size = PHOTO_CHUNK_SIZE;
                    total_chunks = (image_len + chunk_size - 1) / chunk_size;

                    for (chunk_index = 0; chunk_index < total_chunks; chunk_index++) {
                        if (!root_addr_known) {
                            ESP_LOGE(MESH_TAG, "[ML-TX] lost root address at chunk %d", chunk_index);
                            break;
                        }
                        size_t offset = chunk_index * chunk_size;
                        size_t this_chunk = (offset + chunk_size > image_len) ?
                                            (image_len - offset) : chunk_size;

                        photo_packet_t pkt = {
                            .start        = PKT_START,
                            .drone_id     = drone_id,
                            .type         = PKT_TYPE_PHOTO_CHUNK,
                            .chunk_index  = chunk_index,
                            .total_chunks = total_chunks,
                            .data_len     = this_chunk,
                            .end          = PKT_END,
                        };
                        memcpy(pkt.data, image_buf + offset, this_chunk);

                        mesh_data_t data = {
                            .data  = (uint8_t *)&pkt,
                            .size  = sizeof(pkt),
                            .proto = MESH_PROTO_BIN,
                            .tos = MESH_TOS_P2P,
                        };
                        esp_err_t err = esp_mesh_send(&root_addr, &data, MESH_DATA_P2P, NULL, 0);
                        ESP_LOGI(MESH_TAG, "[ML-TX] chunk %d/%d sent err:0x%x",
                                 chunk_index + 1, total_chunks, err);
                        vTaskDelay(10 / portTICK_PERIOD_MS);  
                       // ESP_LOGI(MESH_TAG, "[ML-TX] sending chunk size:%d", sizeof(photo_packet_t));
                        // small delay between chunks
                    }

                    //send photo done
                    packet_t done_pkt = {
                        .start    = PKT_START,
                        .drone_id = drone_id,
                        .type     = PKT_TYPE_PHOTO_DONE,
                        .end      = PKT_END,
                    };
                    memset(done_pkt.payload, 0, sizeof(done_pkt.payload));
                    mesh_data_t done_data = {
                        .data  = (uint8_t *)&done_pkt,
                        .size  = sizeof(done_pkt),
                        .proto = MESH_PROTO_BIN,
                        .tos   = MESH_TOS_P2P,
                    };
                    esp_mesh_send(&root_addr, &done_data, MESH_DATA_P2P, NULL, 0);
                    ESP_LOGI(MESH_TAG, "[ML-TX] photo DONE sent");

                    image_len = 0;
                }
            } 
            else {
                //accumulate image data
                if (image_len + len <= IMAGE_BUF_SIZE) {
                    memcpy(image_buf + image_len, buf, len);
                    image_len += len;
                } else {
                    ESP_LOGE(MESH_TAG, "[ML-RX] image buffer overflow");
                }
            }
        }
    }
}

//mesh Tx task
//modified from ESP-IDF
void esp_mesh_p2p_tx_main(void *arg)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    uint8_t drone_id = mac[5];

    while (1) {
        if (root_addr_known) {
            packet_t pkt = {
                .start    = PKT_START,
                .drone_id = drone_id,
                .end      = PKT_END,
            };
            memset(pkt.payload, 0, sizeof(pkt.payload));

            if (telemetry_valid) {
                pkt.type = PKT_TYPE_TELEMETRY;
                memcpy(pkt.payload, &latest_telemetry, sizeof(telemetry_t));
                ESP_LOGI(MESH_TAG, "[DRONE-TX] telemetry lat:%.5f lon:%.5f alt:%.1f",
                         latest_telemetry.lat, latest_telemetry.lon, latest_telemetry.alt);
            } else {
                pkt.type = PKT_TYPE_HEARTBEAT;
                ESP_LOGI(MESH_TAG, "[DRONE-TX] heartbeat drone_id:%d (no MAVLink yet)", drone_id);
            }

            mesh_data_t data = {
                .data  = (uint8_t *)&pkt,
                .size  = sizeof(pkt),
                .proto = MESH_PROTO_BIN,
                .tos   = MESH_TOS_P2P,
            };
            esp_err_t err = esp_mesh_send(NULL, &data, MESH_DATA_P2P, NULL, 0);
            if (err) {
                ESP_LOGE(MESH_TAG, "[DRONE-TX] send failed err:0x%x", err);
            }
        } else {
            ESP_LOGI(MESH_TAG, "[DRONE-TX] waiting for root address...");
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

//mes Rx task
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

        if (data.size >= sizeof(packet_t)) {
            packet_t *pkt = (packet_t *)data.data;
            if (pkt->start == PKT_START && pkt->end == PKT_END) {
                switch (pkt->type) {

                    case PKT_TYPE_WAYPOINT: {
                        waypoint_t *wp = (waypoint_t *)pkt->payload;
                        ESP_LOGI(MESH_TAG, "[DRONE-RX] waypoint lat:%.5f lon:%.5f alt:%.1f",
                                wp->lat, wp->lon, wp->alt);
                        send_mavlink_waypoint(wp);  // TODO: enable when Pixhawk connected
                        // internal_waypoint_t iwp = { ... };
                        // uart_write_bytes(ML_UART, ...);  // TODO: enable when ML ESP connected
                        break;
                    }

                    case PKT_TYPE_COMMAND: {
                        uint8_t cmd = pkt->payload[0];
                        ESP_LOGI(MESH_TAG, "[DRONE-RX] command type:%d", cmd);
                        send_mavlink_command(cmd);  // TODO: enable when Pixhawk connected
                        break;
                    }

                    case PKT_TYPE_FLAG_ACK: {
                        ESP_LOGI(MESH_TAG, "[DRONE-RX] flag ACK - requesting photo");
                        uart_write_bytes(ML_UART, "REQUEST\n", 8);
                        ml_state = ML_STATE_WAITING_START; //set ML state to skip 'human detected'
                        break;
                    }

                    case PKT_TYPE_ACK: {
                        ack_t *ack = (ack_t *)pkt->payload;
                        ESP_LOGI(MESH_TAG, "[DRONE-RX] ACK type:%d chunk:%d",
                                 ack->ack_type, ack->chunk_index);
                        break;
                    }

                    default:
                        ESP_LOGW(MESH_TAG, "[DRONE-RX] unknown type:%d", pkt->type);
                        break;
                }
            }
        }
    }
    vTaskDelete(NULL);
}

esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
    }
    return ESP_OK;
}

// Event Handler
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
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d", no_parent->scan_times);
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
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d", disconnected->reason);
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
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
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr_event = (mesh_event_root_address_t *)event_data;
        memcpy(&root_addr.addr, root_addr_event->addr, 6);
        root_addr_known = true;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_ADDRESS>root:"MACSTR"", MAC2STR(root_addr.addr));
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
    default:
        ESP_LOGI(MESH_TAG, "unknown event id:%" PRId32 "", event_id);
        break;
    }
}

//app_main
//modified from ESP-IDF
void app_main(void)
{
    // allocate image buffer before WiFi/mesh consumes heap
    image_buf = malloc(IMAGE_BUF_SIZE);
    if (image_buf == NULL) {
        ESP_LOGE(MESH_TAG, "FATAL: failed to allocate image buffer (%d bytes)", IMAGE_BUF_SIZE);
        return;
    }
    ESP_LOGI(MESH_TAG, "image buffer allocated (%d bytes)", IMAGE_BUF_SIZE);

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

    //mesh config — channel 6 matches ground station and GND_ANCHOR
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    memcpy((uint8_t *) &cfg.mesh_id, s_mesh_id, 6);
    cfg.channel = 6;
    char anchor_ssid[] = "GND_ANCHOR";
    cfg.router.ssid_len = strlen(anchor_ssid);
    memcpy((uint8_t *) &cfg.router.ssid, anchor_ssid, cfg.router.ssid_len);
    memcpy((uint8_t *) &cfg.router.password, "gndanchor", strlen("gndanchor"));

    //self-organizing
    // drone finds ground station mesh AP automatically
    ESP_ERROR_CHECK(esp_mesh_set_self_organized(true, false));

    //mesh AP config — allows other drones to connect through this node for multi-hop
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD, strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

    //enable when Pixhawk and ML ESP32 UART
    mavlink_init();
    ml_uart_init();
    xTaskCreate(mavlink_task, "MAVLink", 4096, NULL, 5, NULL);
    xTaskCreate(ml_uart_task, "ML_UART", 4096, NULL, 5, NULL);

    //low capacity ensures drone never wins root election over ground station
    ESP_ERROR_CHECK(esp_mesh_set_capacity_num(10));

    //start mesh
    esp_mesh_set_type(MESH_NODE);
    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGI(MESH_TAG, "DRONE NODE STARTED heap:%" PRId32, esp_get_minimum_free_heap_size());
}