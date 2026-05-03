#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define CAMERA_UART_TXD     (CONFIG_EXAMPLE_UART_TXD)
#define CAMERA_UART_RXD     (CONFIG_EXAMPLE_UART_RXD)
#define CAMERA_UART_RTS     (UART_PIN_NO_CHANGE)
#define CAMERA_UART_CTS     (UART_PIN_NO_CHANGE)

#define CAMERA_UART_PORT    (CONFIG_EXAMPLE_UART_PORT_NUM)
#define CAMERA_UART_BAUD    (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define CAMERA_TASK_STACK   (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define BUF_SIZE (1024)
#define IMAGE_BUF_SIZE (150 * 1024)  // 150KB max image size
#define HUMAN_DETECTED_MSG "human detected!\n"

static const char *TAG = "camera_uart";
static QueueHandle_t uart_queue;
char human_detected_state = 0;

static void camera_uart_task(void *arg)
{
    uart_event_t event;
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    uint8_t *image_buf = (uint8_t *) malloc(IMAGE_BUF_SIZE);
    size_t image_len = 0;
    char receiving_image = 0;

    while (1) {
        // Suspend task until the UART driver signals an event
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(CAMERA_UART_PORT, data, event.size, 100 / portTICK_PERIOD_MS);
                if (len > 0) {
                    if (!human_detected_state) {
                        data[len] = '\0';
                        ESP_LOGI(TAG, "Received: %s", (char *) data);

                        if (strncmp((char *) data, HUMAN_DETECTED_MSG, strlen(HUMAN_DETECTED_MSG)) == 0) {
                            human_detected_state = 1;
                            ESP_LOGI(TAG, "Human detected! Triggering camera.");
                            uart_write_bytes(ECHO_UART_PORT_NUM, "ACK\n", 8);
                        }
                    } else if (!receiving_image) {
                        // Waiting for START_IMAGE
                        data[len] = '\0';
                        if (strncmp((char *) data, "START_IMAGE\n", 12) == 0) {
                            image_len = 0;
                            receiving_image = 1;
                            ESP_LOGI(TAG, "Image transfer started");
                        }
                    } else {
                        // Receiving binary image chunks
                        if (len >= 10 && strncmp((char *) data, "END_IMAGE\n", 10) == 0) {
                            receiving_image = 0;
                            human_detected_state = 0;
                            ESP_LOGI(TAG, "Image received: %d bytes", image_len);
                            // TODO: process image_buf
                        } else {
                            if (image_len + len <= IMAGE_BUF_SIZE) {
                                memcpy(image_buf + image_len, data, len);
                                image_len += len;
                            } else {
                                ESP_LOGE(TAG, "Image buffer overflow");
                            }
                        }
                    }
                }
            }
        }

    }

    free(data);
    free(image_buf);
    vTaskDelete(NULL);
}

void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate  = CAMERA_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(CAMERA_UART_PORT, BUF_SIZE * 2, 0, 0, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(CAMERA_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CAMERA_UART_PORT, CAMERA_UART_TXD, CAMERA_UART_RXD, CAMERA_UART_RTS, CAMERA_UART_CTS));

    xTaskCreate(camera_uart_task, "camera_uart_task", CAMERA_TASK_STACK, NULL, 10, NULL);
}
