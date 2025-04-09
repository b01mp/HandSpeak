#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define SAMPLES 5  // Number of samples for averaging

// Flex Sensor ADC Channels
#define FLEX1 ADC_CHANNEL_5  // GPIO33 - Thumb
#define FLEX2 ADC_CHANNEL_4  // GPIO32 - Index
#define FLEX3 ADC_CHANNEL_7  // GPIO35 - Middle
#define FLEX4 ADC_CHANNEL_6  // GPIO34 - Ring
#define FLEX5 ADC_CHANNEL_3  // GPIO39 - Pinky

// DFPlayer UART Configuration
#define UART_NUM UART_NUM_2
#define TXD_PIN GPIO_NUM_25
#define RXD_PIN GPIO_NUM_26
#define CMD_PLAY_TRACK 0x03

static const char *TAG = "FLEX_DFPLAYER";

adc_oneshot_unit_handle_t adc1_handle;  // ADC Handle

// Function to send commands to DFPlayer
void dfplayer_send_command(uint8_t cmd, uint16_t param) {
    uint8_t packet[10];
    packet[0] = 0x7E;
    packet[1] = 0xFF;
    packet[2] = 0x06;
    packet[3] = cmd;
    packet[4] = 0x00;
    packet[5] = (uint8_t)(param >> 8);
    packet[6] = (uint8_t)(param & 0xFF);
    uint16_t checksum = 0;
    for (int i = 1; i < 7; i++) checksum += packet[i];
    checksum = -checksum;
    packet[7] = (uint8_t)(checksum >> 8);
    packet[8] = (uint8_t)(checksum & 0xFF);
    packet[9] = 0xEF;

    uart_write_bytes(UART_NUM, (const char *)packet, sizeof(packet));
    vTaskDelay(pdMS_TO_TICKS(200));
}

// Function to play specific track
void play_mp3_file(int file_number) {
    if (file_number < 1 || file_number > 5) {
        ESP_LOGE(TAG, "Invalid file number!");
        return;
    }
    ESP_LOGI(TAG, "Playing file %04d.mp3", file_number);
    dfplayer_send_command(CMD_PLAY_TRACK, file_number);
}

// Function to read and average ADC values
int get_smoothed_adc_value(int channel) {
    int sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int value;
        adc_oneshot_read(adc1_handle, channel, &value);
        sum += value;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return sum / SAMPLES;
}

// Gesture detection based on calibrated "straight" ranges
bool is_finger_straight(int value, int min_range, int max_range) {
    return (value >= min_range && value <= max_range);
}

void app_main(void) {
    int last_played = 0;

    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    adc_oneshot_config_channel(adc1_handle, FLEX1, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX2, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX3, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX4, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX5, &config);

    // Initialize UART for DFPlayer
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

    ESP_LOGI(TAG, "Initializing Flex Sensors and DFPlayer...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        int thumb   = get_smoothed_adc_value(FLEX1);
        int index   = get_smoothed_adc_value(FLEX2);
        int middle  = get_smoothed_adc_value(FLEX3);
        int ring    = get_smoothed_adc_value(FLEX4);
        int pinky   = get_smoothed_adc_value(FLEX5);

        ESP_LOGI(TAG, "Thumb: %d, Index: %d, Middle: %d, Ring: %d, Pinky: %d",
                 thumb, index, middle, ring, pinky);

        if (is_finger_straight(index, 1450, 1488) && last_played != 1) {
            play_mp3_file(1);  // Good Morning
            last_played = 1;
        }
        else if (is_finger_straight(thumb, 1470, 1511) && last_played != 2) {
            play_mp3_file(2);
            last_played = 2;
        }
        else if (is_finger_straight(middle, 1770, 1825) && last_played != 3) {
            play_mp3_file(3);
            last_played = 3;
        }
        else if (is_finger_straight(ring, 1710, 1745) && last_played != 4) {
            play_mp3_file(4);
            last_played = 4;
        }
        else if (is_finger_straight(pinky, 1455, 1515) && last_played != 5) {
            play_mp3_file(5);
            last_played = 5;
        }
        else if (!is_finger_straight(thumb, 1470, 1511) &&
                 !is_finger_straight(index, 1450, 1488) &&
                 !is_finger_straight(middle, 1770, 1825) &&
                 !is_finger_straight(ring, 1710, 1745) &&
                 !is_finger_straight(pinky, 1455, 1515)) {
            last_played = 0;  // Reset when no finger is in straight state
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
