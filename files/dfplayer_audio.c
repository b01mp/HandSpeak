/*
    this code plays the audio files inside the dfplayer one after the other
*/

#include <stdio.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define UART_NUM UART_NUM_2
#define TXD_PIN GPIO_NUM_25
#define RXD_PIN GPIO_NUM_26

#define CMD_PLAY_TRACK 0x03
#define CMD_STOP 0x16

static const char *TAG = "DFPLAYER";

void dfplayer_send_command(uint8_t cmd, uint16_t param) {
    uint8_t packet[10];
    packet[0] = 0x7E;  // Start byte
    packet[1] = 0xFF;  // Version
    packet[2] = 0x06;  // Length
    packet[3] = cmd;   // Command
    packet[4] = 0x00;  // Feedback (0x00 = no feedback, 0x01 = feedback)
    packet[5] = (uint8_t)(param >> 8);  // Parameter high byte
    packet[6] = (uint8_t)(param & 0xFF); // Parameter low byte
    // Calculate checksum (sum of bytes 1-6, negated)
    uint16_t checksum = 0;
    for (int i = 1; i < 7; i++) checksum += packet[i];
    checksum = -checksum;
    packet[7] = (uint8_t)(checksum >> 8);  // Checksum high byte
    packet[8] = (uint8_t)(checksum & 0xFF); // Checksum low byte
    packet[9] = 0xEF;  // End byte

    uart_write_bytes(UART_NUM, (const char *)packet, sizeof(packet));
    vTaskDelay(pdMS_TO_TICKS(1000)); // Increased delay for reliability
}

void play_mp3_file(int file_number) {
    if (file_number < 1 || file_number > 5) {
        ESP_LOGE(TAG, "Invalid file number!");
        return;
    }
    ESP_LOGI(TAG, "Playing file %d.mp3", file_number);
    dfplayer_send_command(CMD_PLAY_TRACK, file_number);
}

void app_main(void) {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "Initializing DFPlayer Mini...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Give DFPlayer time to initialize

    ESP_LOGI(TAG, "Playing MP3 files from SD card...");
    for (int i = 1; i <= 5; i++) {
        play_mp3_file(i);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Longer delay to hear each track
    }

    ESP_LOGI(TAG, "Finished playing all files.");
}