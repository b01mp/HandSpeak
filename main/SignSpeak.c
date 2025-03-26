#include <stdio.h>
#include <math.h>
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Audio generation configuration
#define I2S_NUM           I2S_NUM_0
#define I2S_SAMPLE_RATE   44100
#define I2S_SAMPLE_BITS   16
#define I2S_CHANNEL_NUM   2
#define AUDIO_BUFFER_SIZE 1024

// Pin Configuration
#define I2S_BCK_PIN 26
#define I2S_WS_PIN 25
#define I2S_DATA_PIN 22

i2s_chan_handle_t tx_handle;

void init_i2s()
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    i2s_new_channel(&chan_cfg, &tx_handle, NULL);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_PIN ,
            .ws = I2S_WS_PIN,
            .dout = I2S_DATA_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    i2s_channel_init_std_mode(tx_handle, &std_cfg);
    i2s_channel_enable(tx_handle);
}

// Simple sine wave generation function
void generate_sine_wave(int16_t* buffer, size_t buffer_size, float frequency) {
    for (size_t i = 0; i < buffer_size; i++) {
        // Generate sine wave
        float time = (float)i / I2S_SAMPLE_RATE;
        float sample = sin(2 * 3.14 * frequency * time);
        
        // Scale to 16-bit range
        buffer[i] = (int16_t)(sample * 32767);
    }
}

void audio_generation_task(void *pvParameters) {
    // Allocate buffer for audio data
    int16_t* audio_buffer = malloc(AUDIO_BUFFER_SIZE * sizeof(int16_t));
    size_t bytes_written;

    // Configure I2S
    init_i2s();

    while (1) {
        // Generate sine wave at 440 Hz (A4 note)
        generate_sine_wave(audio_buffer, AUDIO_BUFFER_SIZE, 440.0);

        // Write audio data to I2S
        i2s_channel_write(tx_handle, audio_buffer , AUDIO_BUFFER_SIZE * sizeof(int16_t), &bytes_written, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent task flooding
    }
}

void app_main() {
    // Create audio generation task
    xTaskCreate(audio_generation_task, 
                "Audio Generation", 
                4096, 
                NULL, 
                5, 
                NULL);
}