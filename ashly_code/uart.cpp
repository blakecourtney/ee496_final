#include <mavlink/common/mavlink.h>

// Update to match wiring
static const int ESP32_RX_PIN = 16;  // ESP32 RX, Pixhawk TX
static const int ESP32_TX_PIN = 17;  // ESP32 TX, Pixhawk RX (optional)
static const uint32_t MAVLINK_BAUD = 57600; // Pixhawk TELEM baud

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 MAVLink GPS reader starting...");

  Serial2.begin(MAVLINK_BAUD, SERIAL_8N1, ESP32_RX_PIN, ESP32_TX_PIN);
}

void loop() {
  static mavlink_message_t msg;
  static mavlink_status_t status;

  while (Serial2.available() > 0) {
    uint8_t c = (uint8_t)Serial2.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t pos;
        mavlink_msg_global_position_int_decode(&msg, &pos);

        float lat = pos.lat / 1e7f;
        float lon = pos.lon / 1e7f;
        float alt_m = pos.alt / 1000.0f;   // mm -> m

        Serial.print("Lat: ");
        Serial.print(lat, 7);
        Serial.print(" Lon: ");
        Serial.print(lon, 7);
        Serial.print(" Alt: ");
        Serial.print(alt_m, 2);
        Serial.println(" m");
      }
    }
  }
}