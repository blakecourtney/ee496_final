#ifndef CONFIG_H
#define CONFIG_H

// Mesh Network
#define MESH_ID             {0x77, 0x77, 0x77, 0x77, 0x77, 0x77}
#define MESH_CHANNEL        1
#define MESH_MAX_LAYER      6

// BAUD GND station to/from GUI
#define SERIAL_BAUD         115200

// Mesh Packet Types
#define PKT_TYPE_TELEMETRY   1   // drone to GND: GPS, attitude, battery
#define PKT_TYPE_COMMAND     2   // GND to drone: arm, disarm, etc.
#define PKT_TYPE_FLAG        3   // drone to GND: ML person detection alert
#define PKT_TYPE_HEARTBEAT   4   // drone to GND: keep-alive
#define PKT_TYPE_WAYPOINT    5   // GND to drone: search coordinates
#define PKT_TYPE_ACK         7   // GND to drone: generic ack
#define PKT_TYPE_FLAG_ACK    8   // GND to drone: flag received, send photo
#define PKT_TYPE_PHOTO_CHUNK 9   // drone to GND: photo data chunk
#define PKT_TYPE_PHOTO_DONE  10  // drone to GND: all chunks sent
#define PKT_TYPE_PHOTO_START 11  // drone to GND: start photo


// Packet Framing
#define PKT_START            0xFE
#define PKT_END              0xFF

// Mesh Packet Payloads

// Telemetry Packet Payload (drone -> GND)
typedef struct {
    float   lat;
    float   lon;
    float   alt;        // meters above ground level
    float   roll;       // rad
    float   pitch;      // rad
    float   yaw;        // radians
    float   battery;    // volts
    uint8_t satellites;
    uint8_t armed;
} telemetry_t;

//Flag Packet Payload (drone -> GND)
typedef struct {
    float lat;
    float lon;
    float alt;
    float confidence;   // ML confidence 0.0 - 1.0
} flag_t;

//Waypoint Packet Payload (GND -> drone)
typedef struct {
    float lat;
    float lon;
    float alt;          // meters
} waypoint_t;

//ACK Payload (GND -> drone)
typedef struct {
    uint8_t  ack_type;      // PKT_TYPE_FLAG or PKT_TYPE_PHOTO_CHUNK
    uint16_t chunk_index;   // for photo ACK: chunk index; for flag ACK: 0
} ack_t;

//Generic Packet Wrapper (all packets except photo)
typedef struct {
    uint8_t start;          // PKT_START (0xFE)
    uint8_t drone_id;
    uint8_t type;           // PKT_TYPE_*
    uint8_t payload[64];
    uint8_t end;            // PKT_END (0xFF)
} packet_t;

// Photo Packet
#define PHOTO_CHUNK_SIZE     400 

typedef struct {
    uint8_t  start;
    uint8_t  drone_id;
    uint8_t  type;              // PKT_TYPE_PHOTO_CHUNK
    uint16_t chunk_index;
    uint16_t total_chunks;
    uint16_t data_len;
    uint8_t  data[PHOTO_CHUNK_SIZE];
    uint8_t  end;               // PKT_END (0xFF)
} photo_packet_t;

// BAUD esp cam to network esp
#define INTERNAL_BAUD               115200

// esp-cam to network esp
#define INTERNAL_PKT_FLAG           0x01  // person detected
#define INTERNAL_PKT_IMG_CHUNK      0x02  // image chunk
#define INTERNAL_PKT_IMG_DONE       0x03  // all chunks sent

// netowrk esp to esp-cam
#define INTERNAL_PKT_PHOTO_REQUEST  0x04  // GND wants photo
#define INTERNAL_PKT_WAYPOINT       0x05  // new search coordinates

typedef struct {
    uint8_t type;           // INTERNAL_PKT_FLAG
    float   confidence;     // 0.0 - 1.0
} internal_flag_t;

typedef struct {
    uint8_t  type;          // INTERNAL_PKT_IMG_CHUNK
    uint16_t chunk_index;
    uint16_t total_chunks;
    uint16_t data_len;
    uint8_t  data[PHOTO_CHUNK_SIZE];
} internal_img_chunk_t;

typedef struct {
    uint8_t type;           // INTERNAL_PKT_WAYPOINT
    float   lat;
    float   lon;
    float   alt;
} internal_waypoint_t;

#endif // CONFIG_H