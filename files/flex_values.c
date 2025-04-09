/*
code for getting adc values from the flex sensors

FINAL CODE
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

#define SAMPLES 50
#define FLEX1 ADC_CHANNEL_5  // GPIO33
#define FLEX2 ADC_CHANNEL_4  // GPIO32
#define FLEX3 ADC_CHANNEL_7  // GPIO35
#define FLEX4 ADC_CHANNEL_6  // GPIO34
#define FLEX5 ADC_CHANNEL_3  // GPIO39

static const char *TAG = "FLEX_SENSOR";

adc_oneshot_unit_handle_t adc1_handle;

int get_smoothed_adc_value(int channel) {
    int sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int value;
        adc_oneshot_read(adc1_handle, channel, &value);
        sum += value;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return sum / SAMPLES;
}

void app_main(void) {
    int prev_values[5] = {-1, -1, -1, -1, -1};  // Previous values for each sensor

    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    // Configure ADC Channels
    adc_oneshot_chan_cfg_t config = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    adc_oneshot_config_channel(adc1_handle, FLEX1, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX2, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX3, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX4, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX5, &config);

    ESP_LOGI(TAG, "Starting Flex Sensor Test on GPIO33, 32, 35, 34, 39...");

    while (1) {
        int values[5];
        values[0] = get_smoothed_adc_value(FLEX1);
        values[1] = get_smoothed_adc_value(FLEX2);
        values[2] = get_smoothed_adc_value(FLEX3);
        values[3] = get_smoothed_adc_value(FLEX4);
        values[4] = get_smoothed_adc_value(FLEX5);

        // Check each sensor for significant change
        for (int i = 0; i < 5; i++) {
            if (abs(values[i] - prev_values[i]) > 10 || prev_values[i] == -1) {
                ESP_LOGI(TAG, "Flex%d Value: %d", i + 1, values[i]);
                prev_values[i] = values[i];  // Update previous value
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}