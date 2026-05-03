#include <Arduino.h>
#include "esp_camera.h"
#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "drone_model_quant.h"

uint8_t *tensor_arena = NULL;
const int arena_size = 1024 * 1024;

// Freenove ESP32-S3 Pin Map
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

// Hardware Serial for Board-to-Board Communication
HardwareSerial DroneSerial(1);
#define MESH_TX_PIN 47
#define MESH_RX_PIN 21

#define MAX_INPUT 10

// TFLite Globals
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
tflite::ErrorReporter* error_reporter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;

float detect_thresh = 0.1;
uint8_t detect_count = 0;
char detect_flag = 0;
char detect_ack_flag = 0;
bool waiting_for_ack = false;
char request_flag = 0;

uint8_t *frame_capture = NULL; 
size_t capture_len = 0;
float probability_capture = 0;
float best_probability = 0;

// Function prototype so the compiler knows it exists
void processSerialByte(const byte inByte);

void setup() {
  Serial.begin(115200);
  DroneSerial.begin(115200, SERIAL_8N1, MESH_RX_PIN, MESH_TX_PIN);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Camera Init
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_240X240;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  if (esp_camera_init(&config) != ESP_OK) {
      return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);
      s->set_hmirror(s, 1);
      s->set_brightness(s, 1);
  }

  // TFLite Init
  tensor_arena = (uint8_t *)heap_caps_aligned_alloc(16, arena_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (tensor_arena == NULL) return;

  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;
  model = tflite::GetModel(g_model_data);
  static tflite::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, arena_size, error_reporter);

  interpreter = &static_interpreter;
  if (interpreter->AllocateTensors() != kTfLiteOk) return;
  
  input = interpreter->input(0);
  output = interpreter->output(0);
}

void loop() {
  // --- CAMERA & INFERENCE LOGIC ---
  if (!detect_flag){
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) return;

    int8_t* input_ptr = input->data.int8;
    int total_pixels = 240 * 240;

    for (int i = 0; i < total_pixels; i++) {
      uint16_t pixel = (fb->buf[i * 2] << 8) | fb->buf[i * 2 + 1];
      uint8_t r = ((pixel >> 11) & 0x1F) << 3;
      uint8_t g = ((pixel >> 5)  & 0x3F) << 2;
      uint8_t b = (pixel & 0x1F)       << 3;
      input_ptr[i * 3 + 0] = (int8_t)(r - 128);
      input_ptr[i * 3 + 1] = (int8_t)(g - 128);
      input_ptr[i * 3 + 2] = (int8_t)(b - 128);
    }

    if (interpreter->Invoke() == kTfLiteOk) {
      int8_t raw_score = output->data.int8[0];
      float score = (raw_score + 128) / 255.0f;
      // MODEL DEBUG
      // Serial.printf("input type: %d (uint8=3, int8=9)\n", input->type);
      // Serial.printf("input params: scale=%.6f zp=%d\n", input->params.scale, input->params.zero_point);
      // Serial.printf("output params: scale=%.6f zp=%d\n", output->params.scale, output->params.zero_point);
      // Serial.printf("raw_score: %d\n", (int)raw_score);
      // Serial.printf("sample input r=%d g=%d b=%d\n", (int)input->data.uint8[0], (int)input->data.uint8[1], (int)input->data.uint8[2]);

      // MODEL VISUALATION DEBUG START
      // Serial.println("START_IMAGE");
      // size_t chunk_size = 2048;
      // uint8_t *buffer_ptr = fb->buf;
      // size_t total_bytes = fb->len;

      // for (size_t i = 0; i < total_bytes; i += chunk_size) {
      //   size_t bytes_to_send = (total_bytes - i < chunk_size) ? (total_bytes - i) : chunk_size;
      //   Serial.write(buffer_ptr + i, bytes_to_send);
      //   Serial.flush();
      // }
      // Serial.println("END_IMAGE");
      Serial.printf("PROBABILITY: %.4f\n", score);
      Serial.printf("HUMAN DETECTED: %d\n", (score > detect_thresh));
      // MODEL VISUALATION DEBUG END 

      if (request_flag){
        Serial.println("Sending requested\n");
        digitalWrite(LED_BUILTIN, HIGH);
        DroneSerial.println("START_IMAGE");
        size_t chunk_size = 2048;
        uint8_t *buffer_ptr = fb->buf;
        size_t total_bytes = fb->len;

        for (size_t i = 0; i < total_bytes; i += chunk_size) {
          size_t bytes_to_send = (total_bytes - i < chunk_size) ? (total_bytes - i) : chunk_size;
          DroneSerial.write(buffer_ptr + i, bytes_to_send);
          DroneSerial.flush();
        }
        DroneSerial.println("END_IMAGE");
        digitalWrite(LED_BUILTIN, LOW);
        request_flag = 0;
      }

      if (score > detect_thresh)
        detect_count += 1;

      if (score > probability_capture){
        if (frame_capture != NULL) {
          free(frame_capture);
          frame_capture = NULL;
        }
        frame_capture = (uint8_t *)ps_malloc(fb->len);
        if (!frame_capture) { 
          esp_camera_fb_return(fb);
          return;
        }
        memcpy(frame_capture, fb->buf, fb->len);
        capture_len = fb->len;      
        probability_capture = score;
        best_probability = score;   
      }

      if (detect_count == 2){
        detect_flag = 1;
        detect_count = 0;
        Serial.println("HELLO\n");
      }
    }

    esp_camera_fb_return(fb);
  }

  // --- UART COMMUNICATION LOGIC ---

  if (detect_flag == 1 && !waiting_for_ack){
    DroneSerial.println("human detected!");
    digitalWrite(LED_BUILTIN, HIGH);
    waiting_for_ack = true; 
    digitalWrite(LED_BUILTIN, HIGH);
  }

  while (DroneSerial.available() > 0) {
    processSerialByte(DroneSerial.read());
  }
  if (detect_ack_flag){
    digitalWrite(LED_BUILTIN, HIGH);
    DroneSerial.println("START_IMAGE");
    size_t chunk_size = 2048;
    uint8_t *buffer_ptr = frame_capture; 

    for (size_t i = 0; i < capture_len; i += chunk_size) { 
      size_t bytes_to_send = (capture_len - i < chunk_size) ? (capture_len - i) : chunk_size;
      DroneSerial.write(buffer_ptr + i, bytes_to_send);
      DroneSerial.flush();
    }
    DroneSerial.println("END_IMAGE");
    DroneSerial.printf("PROBABILITY: %.2f\n", best_probability);
    
    detect_ack_flag = 0;      
  }
}

void processSerialByte(const byte inByte) {
  static char input_line[MAX_INPUT];
  static unsigned int input_pos = 0;

  Serial.println(inByte);

  switch (inByte){
    case '\n':
      input_line[input_pos] = 0;

      if (strcmp(input_line, "ACK") == 0){  
        detect_ack_flag = 1;
        waiting_for_ack = false;  
        digitalWrite(LED_BUILTIN, LOW);
      }
      if (strcmp(input_line, "CLEAR") == 0){ 
        detect_flag = 0;
        probability_capture = 0; 
        digitalWrite(LED_BUILTIN, LOW);
        delay(5000);
        Serial.println("Hi2\n");
      }
      if (strcmp(input_line, "REQUEST") == 0){ 
        request_flag = 1;
      }

      input_pos = 0;
      break;

    case '\r':
      // Ignore carriage returns
      break;

    default:
      if (input_pos < (MAX_INPUT - 1))
        input_line[input_pos++] = inByte;
      break;
  }
}