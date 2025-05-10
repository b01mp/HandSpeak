#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "math.h"

#define I2C_MASTER_SCL_IO    22
#define I2C_MASTER_SDA_IO    21
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   50000

#define MPU6050_ADDR         0x68
#define ACCEL_XOUT_H         0x3B
#define GYRO_XOUT_H          0x43
#define PWR_MGMT_1           0x6B
#define WHO_AM_I             0x75
#define CONFIG               0x1A
#define GYRO_CONFIG          0x1B
#define ACCEL_CONFIG         0x1C

static const char *TAG = "MPU6050";

// Calibration offsets
int16_t gyro_offset_x = 0;
int16_t gyro_offset_y = 0;
int16_t gyro_offset_z = 0;

// Low-pass filter coefficients
#define ALPHA 0.8f  // Adjust between 0-1 (higher = more smoothing)
float filtered_x = 0;
float filtered_y = 0;
float filtered_z = 0;

static esp_err_t i2c_master_init(void)
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

// Helper for writing a byte to MPU6050 register
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

// Helper for reading a byte from MPU6050 register
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

void i2c_scanner() {
    printf("Scanning I2C bus...\n");
    uint8_t address;
    int devices_found = 0;
    
    for (address = 1; address < 127; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            printf("Device found at address 0x%02X\n", address);
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        printf("No I2C devices found\n");
    } else {
        printf("Scan complete, found %d device(s)\n", devices_found);
    }
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
    vTaskDelay(250 / portTICK_PERIOD_MS);
    
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
    // Value 0x04 sets bandwidth to 20Hz (smooths data but adds 8.5ms delay)
    ret = mpu6050_write_reg(CONFIG, 0x04);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setting DLPF failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure gyroscope (±250deg/s full scale range)
    ret = mpu6050_write_reg(GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setting gyro scale failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure accelerometer (±2g full scale range)
    ret = mpu6050_write_reg(ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setting accel scale failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
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
    
    // Log raw and processed values
    ESP_LOGI(TAG, "Raw - Gyro  X: %6d, Y: %6d, Z: %6d", raw_x, raw_y, raw_z);
    ESP_LOGI(TAG, "Gyro (deg/s) - X: %.2f, Y: %.2f, Z: %.2f", *gyro_x, *gyro_y, *gyro_z);
    
    if(filtered_z > 25)
    {
        ESP_LOGI(TAG,"HELLO");
    }
    if(filtered_x > 20)
    {
        ESP_LOGI(TAG,"GOD B;ESS U");
    }
    if(filtered_y > 20)
    {
        ESP_LOGI(TAG,"HOOOOOOO");
    }


    
    return ESP_OK;
}

void app_main(void)
{
    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    i2c_scanner();

    // Initialize MPU6050
    ESP_ERROR_CHECK(mpu6050_init());
    
    // Calibrate gyroscope (100 samples)
    ESP_ERROR_CHECK(mpu6050_calibrate_gyro(100));
    
    // Main loop - read and process gyroscope data
    float gyro_x, gyro_y, gyro_z;
    while(1) {
        if (mpu6050_read_gyro(&gyro_x, &gyro_y, &gyro_z) == ESP_OK) {
            // Data is already logged in the read function
        } else {
            ESP_LOGE(TAG, "Failed to read gyroscope data");
        }
        
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}