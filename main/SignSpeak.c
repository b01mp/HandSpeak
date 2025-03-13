#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

#define FLEX_SENSOR_PIN ADC_CHANNEL_6  // GPIO34

static const char *TAG = "FLEX_SENSOR";

void app_main(void) {
    adc_oneshot_unit_handle_t adc1_handle;
    
    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    // Configure ADC Channel
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };
    adc_oneshot_config_channel(adc1_handle, FLEX_SENSOR_PIN, &config);

    while (1) {
        int flex_value;
        adc_oneshot_read(adc1_handle, FLEX_SENSOR_PIN, &flex_value);
        ESP_LOGI(TAG, "Flex Sensor ADC Value: %d", flex_value*2);
        flex_value = flex_value*4;
        if (flex_value > 5000 && flex_value < 5400){
            ESP_LOGI(TAG,"Hey there,How are you");
        }

        else if(flex_value > 5400 && flex_value < 5800)
        {
            ESP_LOGI(TAG,"Hoo there,How are you");
        } 

        else{
            ESP_LOGI(TAG,"Hoooo there,Whoooo are you");
        }



        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
