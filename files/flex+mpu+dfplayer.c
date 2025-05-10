#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "math.h"

// I2C Configuration for MPU6050
#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   50000

// MPU6050 Registers
#define MPU6050_ADDR         0x68
#define ACCEL_XOUT_H         0x3B
#define GYRO_XOUT_H          0x43
#define PWR_MGMT_1           0x6B
#define WHO_AM_I             0x75
#define CONFIG               0x1A
#define GYRO_CONFIG          0x1B
#define ACCEL_CONFIG         0x1C

// Flex Sensor ADC Channels
#define SAMPLES              20    // Number of samples for averaging
#define FLEX1 ADC_CHANNEL_3  // GPIO33 - Thumb   3
#define FLEX2 ADC_CHANNEL_6  // GPIO32 - Index   6
#define FLEX3 ADC_CHANNEL_7  // GPIO35 - Middle  7
#define FLEX4 ADC_CHANNEL_4  // GPIO34 - Ring    4
#define FLEX5 ADC_CHANNEL_5  // GPIO39 - Pinky   5

// DFPlayer UART Configuration
#define UART_NUM             UART_NUM_2
#define TXD_PIN              GPIO_NUM_25
#define RXD_PIN              GPIO_NUM_26
#define CMD_PLAY_TRACK       0x03

// Gesture State Machine
#define GESTURE_NONE         0
#define GESTURE_FLEX_INDEX   1  // Index finger flex
#define GESTURE_FLEX_THUMB   2  // Thumb flex
#define GESTURE_FLEX_PINKY   3  // Pinky finger flex
#define GESTURE_GYRO_RIGHT   4  // Wrist turn right
#define GESTURE_GYRO_LEFT    5  // Wrist turn left

static const char *TAG = "GESTURE_AUDIO";

// Global variables for sensors
adc_oneshot_unit_handle_t adc1_handle;  // ADC Handle

// Calibration offsets
int16_t gyro_offset_x = 0;
int16_t gyro_offset_y = 0;
int16_t gyro_offset_z = 0;

// Low-pass filter coefficients for gyro
#define ALPHA 0.8f  // Adjust between 0-1 (higher = more smoothing)
float filtered_x = 0;
float filtered_y = 0;
float filtered_z = 0;

// Gesture detection variables
int current_gesture = GESTURE_NONE;
int last_played = 0;
uint32_t gesture_start_time = 0;
#define GESTURE_COOLDOWN_MS 1000  // Cooldown between gestures

// Function prototypes - Removed 'static' conflict
esp_err_t i2c_master_init(void);
esp_err_t mpu6050_init(void);
esp_err_t mpu6050_calibrate_gyro(uint16_t samples);
esp_err_t mpu6050_read_gyro(float *gyro_x, float *gyro_y, float *gyro_z);
void dfplayer_init(void);
void adc_init(void);
void play_mp3_file(int file_number);

//======================== MPU6050 FUNCTIONS ========================

// Initialize I2C master - Removed 'static' to match prototype
esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Write a byte to MPU6050 register
esp_err_t mpu6050_write_reg(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read a byte from MPU6050 register
esp_err_t mpu6050_read_reg(uint8_t reg_addr, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

// Read multiple bytes from MPU6050
esp_err_t mpu6050_read_bytes(uint8_t reg_addr, uint8_t *buffer, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

    if (size > 1) {
        i2c_master_read(cmd, buffer, size - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buffer + size - 1, I2C_MASTER_NACK);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

// Initialize MPU6050
esp_err_t mpu6050_init(void)
{
    esp_err_t ret;
    uint8_t check;

    // Reset the device
    ret = mpu6050_write_reg(PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device reset failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Give time for reset to complete
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Wake up the device
    ret = mpu6050_write_reg(PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device wake up failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Check device ID
    ret = mpu6050_read_reg(WHO_AM_I, &check);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (check != MPU6050_ADDR) {
        ESP_LOGE(TAG, "Wrong device ID: 0x%02X (expected 0x%02X)", check, MPU6050_ADDR);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MPU6050 found with ID: 0x%02X", check);

    // Configure digital low pass filter (DLPF)
    ret = mpu6050_write_reg(CONFIG, 0x04);  // 20Hz bandwidth
    if (ret != ESP_OK) return ret;

    // Configure gyroscope (±250deg/s full scale range)
    ret = mpu6050_write_reg(GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    // Configure accelerometer (±2g full scale range)
    ret = mpu6050_write_reg(ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

// Calibrate gyroscope by taking multiple readings when device is stationary
esp_err_t mpu6050_calibrate_gyro(uint16_t samples)
{
    ESP_LOGI(TAG, "Calibrating gyroscope - keep the sensor still...");

    int32_t x_sum = 0, y_sum = 0, z_sum = 0;
    uint8_t data[6];
    esp_err_t ret;

    // Take multiple readings and average them
    for (int i = 0; i < samples; i++) {
        ret = mpu6050_read_bytes(GYRO_XOUT_H, data, 6);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Calibration read failed: %s", esp_err_to_name(ret));
            return ret;
        }

        int16_t gyro_x = (int16_t)((data[0] << 8) | data[1]);
        int16_t gyro_y = (int16_t)((data[2] << 8) | data[3]);
        int16_t gyro_z = (int16_t)((data[4] << 8) | data[5]);

        x_sum += gyro_x;
        y_sum += gyro_y;
        z_sum += gyro_z;

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Calculate average offsets
    gyro_offset_x = x_sum / samples;
    gyro_offset_y = y_sum / samples;
    gyro_offset_z = z_sum / samples;

    ESP_LOGI(TAG, "Gyro offsets - X: %d, Y: %d, Z: %d", 
             gyro_offset_x, gyro_offset_y, gyro_offset_z);

    return ESP_OK;
}

// Read gyroscope data with calibration and filtering
esp_err_t mpu6050_read_gyro(float *gyro_x, float *gyro_y, float *gyro_z)
{
    uint8_t data[6];
    esp_err_t ret = mpu6050_read_bytes(GYRO_XOUT_H, data, 6);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Reading gyro data failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Extract raw values
    int16_t raw_x = (int16_t)((data[0] << 8) | data[1]) - gyro_offset_x;
    int16_t raw_y = (int16_t)((data[2] << 8) | data[3]) - gyro_offset_y;
    int16_t raw_z = (int16_t)((data[4] << 8) | data[5]) - gyro_offset_z;

    // Convert to deg/s (for ±250 deg/s range, LSB sensitivity is 131 LSB/deg/s)
    float x_deg_s = raw_x / 131.0f;
    float y_deg_s = raw_y / 131.0f;
    float z_deg_s = raw_z / 131.0f;

    // Apply low-pass filter
    filtered_x = ALPHA * filtered_x + (1 - ALPHA) * x_deg_s;
    filtered_y = ALPHA * filtered_y + (1 - ALPHA) * y_deg_s;
    filtered_z = ALPHA * filtered_z + (1 - ALPHA) * z_deg_s;

    // Apply threshold to reduce noise
    const float threshold = 0.5f; // Degrees per second
    if (fabs(filtered_x) < threshold) filtered_x = 0;
    if (fabs(filtered_y) < threshold) filtered_y = 0;
    if (fabs(filtered_z) < threshold) filtered_z = 0;

    // Return processed values
    *gyro_x = filtered_x;
    *gyro_y = filtered_y;
    *gyro_z = filtered_z;

    return ESP_OK;
}

//======================== FLEX SENSOR FUNCTIONS ========================

// Initialize ADC for flex sensors
void adc_init(void)
{
    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    adc_oneshot_config_channel(adc1_handle, FLEX1, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX2, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX3, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX4, &config);
    adc_oneshot_config_channel(adc1_handle, FLEX5, &config);

    ESP_LOGI(TAG, "ADC initialized for flex sensors");
}

// Function to read and average ADC values
int get_smoothed_adc_value(int channel) {
    int sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        int value;
        adc_oneshot_read(adc1_handle, channel, &value);
        sum += value;
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Reduced delay for faster response
    }
    return sum / SAMPLES;
}

// Gesture detection based on calibrated "straight" ranges
bool is_finger_straight(int value, int min_range, int max_range) {
    return (value >= min_range && value <= max_range);
}

//======================== DFPLAYER FUNCTIONS ========================

// Initialize DFPlayer Mini
void dfplayer_init(void)
{
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

    ESP_LOGI(TAG, "DFPlayer initialized");
    vTaskDelay(pdMS_TO_TICKS(1000));  // Give DFPlayer time to initialize
}

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

//======================== GESTURE DETECTION ========================

// Function to detect gestures based on both flex sensors and gyroscope data
void detect_gestures(float gyro_x, float gyro_y, float gyro_z) {
    // Read flex sensor values
    int thumb  = get_smoothed_adc_value(FLEX1);
    int index  = get_smoothed_adc_value(FLEX2);
    int pinky = get_smoothed_adc_value(FLEX5);
    
    // Timestamp for gesture cooldown
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check if enough time has passed since the last gesture
    bool can_trigger = (current_time - gesture_start_time > GESTURE_COOLDOWN_MS);
    
    if (!can_trigger) {
        return; // Still in cooldown period
    }
    
    // Detect gestures and play corresponding audio
    int detected_gesture = GESTURE_NONE;
    
    // Gesture 1: Index finger flex
    if (is_finger_straight(index, 1390, 1430)) {
        detected_gesture = GESTURE_FLEX_INDEX;
    }
    // Gesture 2: Thumb flex
    else if (is_finger_straight(thumb, 1400, 1450)) {
        detected_gesture = GESTURE_FLEX_THUMB;
    }
    // Gesture 3: Wrist rotation right (z-axis)
    else if (gyro_z > 20) {
        detected_gesture = GESTURE_GYRO_RIGHT;
    }
    // Gesture 4: Wrist rotation left (z-axis)
    else if (gyro_z < -20) {
        detected_gesture = GESTURE_GYRO_LEFT;
    }
    // Gesture 5: Middle finger flex
    else if (is_finger_straight(pinky, 1720, 1780)) {
        detected_gesture = GESTURE_FLEX_PINKY;
    }
    
    // Play the corresponding audio file if a new gesture is detected
    if (detected_gesture != GESTURE_NONE && detected_gesture != current_gesture) {
        current_gesture = detected_gesture;
        gesture_start_time = current_time;
        
        // Map gesture to audio file
        switch (detected_gesture) {
            case GESTURE_FLEX_INDEX:
                play_mp3_file(1);  // Good Morning
                ESP_LOGI(TAG, "Gesture: Index finger flex - Playing 'Good Morning'");
                break;
                
            case GESTURE_FLEX_THUMB:
                play_mp3_file(2);  // Bye Bye
                ESP_LOGI(TAG, "Gesture: Thumb flex - Playing 'Bye Bye'");
                break;
                
            case GESTURE_FLEX_PINKY:
                play_mp3_file(3);  // Good Night
                ESP_LOGI(TAG, "Gesture: Middle finger flex - Playing 'Good Night'");
                break;
                
            case GESTURE_GYRO_RIGHT:
                play_mp3_file(4);  // Thank You
                ESP_LOGI(TAG, "Gesture: Wrist turn right - Playing 'Thank You'");
                break;
                
            case GESTURE_GYRO_LEFT:
                play_mp3_file(5);  // Hello
                ESP_LOGI(TAG, "Gesture: Wrist turn left - Playing 'Hello'");
                break;
        }
    }
    else if (detected_gesture == GESTURE_NONE) {
        // Reset current gesture when no gesture is detected
        current_gesture = GESTURE_NONE;
    }
}

//======================== MAIN APPLICATION ========================

void app_main(void)
{
    // Initialize I2C for MPU6050
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    // Initialize MPU6050
    ESP_ERROR_CHECK(mpu6050_init());
    
    // Initialize ADC for flex sensors
    adc_init();
    
    // Initialize DFPlayer Mini
    dfplayer_init();
    
    // Calibrate gyroscope (100 samples)
    ESP_ERROR_CHECK(mpu6050_calibrate_gyro(100));
    
    ESP_LOGI(TAG, "Initialization complete! Ready for gesture detection.");
    
    // Main loop - read sensors and detect gestures
    float gyro_x, gyro_y, gyro_z;

    
    
    while(1) {
        // Read gyroscope data
        if (mpu6050_read_gyro(&gyro_x, &gyro_y, &gyro_z) == ESP_OK) {
            // Log gyro data occasionally for debugging
            ESP_LOGD(TAG, "Gyro (deg/s) - X: %.2f, Y: %.2f, Z: %.2f", gyro_x, gyro_y, gyro_z);
            
            // Detect gestures based on both sensor types
            detect_gestures(gyro_x, gyro_y, gyro_z);
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);  // 50ms loop for responsive gesture detection
    }
}