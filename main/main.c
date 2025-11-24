/*
 * MIT License
 *
 * Copyright (c) 2025 Adam G. Sweeney <agsweeney@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "lsm6ds3.h"
#include "lsm6ds3_fusion.h"

static const char *TAG = "LSM6DS3_EXAMPLE";

#define I2C_MASTER_SCL_IO           8
#define I2C_MASTER_SDA_IO           7
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define LSM6DS3_I2C_ADDR            0x6A

#define GPIO_ROLL_ZERO              4
#define GPIO_PITCH_ZERO             5
#define GPIO_YAW_ZERO               18

static lsm6ds3_handle_t sensor_handle;
static i2c_master_bus_handle_t i2c_bus_handle;
static lsm6ds3_angle_zero_t angle_zero_ref;
static lsm6ds3_euler_angles_t current_euler;
static SemaphoreHandle_t data_mutex = NULL;
static TickType_t last_nvs_save_time = 0;
static bool nvs_save_pending = false;
static const TickType_t NVS_SAVE_MIN_INTERVAL_MS = 1000;

static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));
    
    return ESP_OK;
}

static void gpio_zero_task(void *pvParameters)
{
    bool roll_pressed = false;
    bool pitch_pressed = false;
    bool yaw_pressed = false;
    
    while (1) {
        bool roll_state = !gpio_get_level(GPIO_ROLL_ZERO);
        bool pitch_state = !gpio_get_level(GPIO_PITCH_ZERO);
        bool yaw_state = !gpio_get_level(GPIO_YAW_ZERO);
        
        TickType_t current_time = xTaskGetTickCount();
        // TickType_t is unsigned, so subtraction naturally handles wrap-around
        bool nvs_save_allowed = (current_time - last_nvs_save_time) >= pdMS_TO_TICKS(NVS_SAVE_MIN_INTERVAL_MS);
        
        if (roll_state && !roll_pressed) {
            roll_pressed = true;
            lsm6ds3_euler_angles_t euler_copy;
            lsm6ds3_angle_zero_t zero_ref_copy;
            
            if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                euler_copy = current_euler;
                lsm6ds3_set_roll_zero(&angle_zero_ref, euler_copy.roll);
                zero_ref_copy = angle_zero_ref;
                xSemaphoreGive(data_mutex);
                
                if (nvs_save_allowed) {
                    lsm6ds3_save_angle_zero_to_nvs(&zero_ref_copy, "lsm6ds3");
                    last_nvs_save_time = current_time;
                    nvs_save_pending = false;
                    ESP_LOGI(TAG, "Roll zero set to %.2f°", euler_copy.roll);
                } else {
                    nvs_save_pending = true;
                    ESP_LOGW(TAG, "Roll zero set to %.2f° (NVS save rate limited, will save later)", euler_copy.roll);
                }
            }
        } else if (!roll_state) {
            roll_pressed = false;
        }
        
        if (pitch_state && !pitch_pressed) {
            pitch_pressed = true;
            lsm6ds3_euler_angles_t euler_copy;
            lsm6ds3_angle_zero_t zero_ref_copy;
            
            if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                euler_copy = current_euler;
                lsm6ds3_set_pitch_zero(&angle_zero_ref, euler_copy.pitch);
                zero_ref_copy = angle_zero_ref;
                xSemaphoreGive(data_mutex);
                
                if (nvs_save_allowed) {
                    lsm6ds3_save_angle_zero_to_nvs(&zero_ref_copy, "lsm6ds3");
                    last_nvs_save_time = current_time;
                    nvs_save_pending = false;
                    ESP_LOGI(TAG, "Pitch zero set to %.2f°", euler_copy.pitch);
                } else {
                    nvs_save_pending = true;
                    ESP_LOGW(TAG, "Pitch zero set to %.2f° (NVS save rate limited, will save later)", euler_copy.pitch);
                }
            }
        } else if (!pitch_state) {
            pitch_pressed = false;
        }
        
        if (yaw_state && !yaw_pressed) {
            yaw_pressed = true;
            lsm6ds3_euler_angles_t euler_copy;
            lsm6ds3_angle_zero_t zero_ref_copy;
            
            if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                euler_copy = current_euler;
                lsm6ds3_set_yaw_zero(&angle_zero_ref, euler_copy.yaw);
                zero_ref_copy = angle_zero_ref;
                xSemaphoreGive(data_mutex);
                
                if (nvs_save_allowed) {
                    lsm6ds3_save_angle_zero_to_nvs(&zero_ref_copy, "lsm6ds3");
                    last_nvs_save_time = current_time;
                    nvs_save_pending = false;
                    ESP_LOGI(TAG, "Yaw zero set to %.2f°", euler_copy.yaw);
                } else {
                    nvs_save_pending = true;
                    ESP_LOGW(TAG, "Yaw zero set to %.2f° (NVS save rate limited, will save later)", euler_copy.yaw);
                }
            }
        } else if (!yaw_state) {
            yaw_pressed = false;
        }
        
        if (nvs_save_pending && nvs_save_allowed) {
            lsm6ds3_angle_zero_t zero_ref_copy;
            if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                zero_ref_copy = angle_zero_ref;
                xSemaphoreGive(data_mutex);
                
                lsm6ds3_save_angle_zero_to_nvs(&zero_ref_copy, "lsm6ds3");
                last_nvs_save_time = current_time;
                nvs_save_pending = false;
                ESP_LOGI(TAG, "Pending angle zero saved to NVS");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void sensor_task(void *pvParameters)
{
    lsm6ds3_madgwick_filter_t madgwick_filter;
    lsm6ds3_complementary_filter_t comp_filter;
    
    float accel_mg[3];
    float gyro_mdps[3];
    float gyro_dps[3];
    lsm6ds3_euler_angles_t euler;
    float comp_roll, comp_pitch;
    
    const float sample_rate_hz = 104.0f;
    const float dt = 1.0f / sample_rate_hz;
    
    ESP_LOGI(TAG, "Initializing sensor fusion filters...");
    
    lsm6ds3_madgwick_init(&madgwick_filter, 0.1f, sample_rate_hz);
    lsm6ds3_complementary_init(&comp_filter, 0.98f, sample_rate_hz);
    
    ESP_LOGI(TAG, "Starting sensor data reading loop...");
    ESP_LOGI(TAG, "Sample rate: %.1f Hz", sample_rate_hz);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Format: Accel[mg] | Gyro[dps] | Madgwick[deg] | Complementary[deg] | Ground");
    ESP_LOGI(TAG, "       X    Y    Z  |  X    Y    Z  | Roll Pitch Yaw | Roll Pitch | Angle");
    ESP_LOGI(TAG, "----------------------------------------------------------------------------");
    ESP_LOGI(TAG, "GPIO %d=Roll Zero, GPIO %d=Pitch Zero, GPIO %d=Yaw Zero", 
             GPIO_ROLL_ZERO, GPIO_PITCH_ZERO, GPIO_YAW_ZERO);
    ESP_LOGI(TAG, "Ground Angle: 0°=vertical up, 90°=horizontal, 180°=vertical down");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t sample_count = 0;
    
    while (1) {
        esp_err_t ret;
        
        ret = lsm6ds3_read_accel(&sensor_handle, accel_mg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read accelerometer");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        ret = lsm6ds3_read_gyro(&sensor_handle, gyro_mdps);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read gyroscope");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        gyro_dps[0] = gyro_mdps[0] / 1000.0f;
        gyro_dps[1] = gyro_mdps[1] / 1000.0f;
        gyro_dps[2] = gyro_mdps[2] / 1000.0f;
        
        lsm6ds3_madgwick_update(&madgwick_filter, accel_mg, gyro_dps, dt);
        lsm6ds3_madgwick_get_euler(&madgwick_filter, &euler);
        
        lsm6ds3_complementary_update(&comp_filter, accel_mg, gyro_dps, dt);
        lsm6ds3_complementary_get_angles(&comp_filter, &comp_roll, &comp_pitch);
        
        lsm6ds3_angle_zero_t angle_zero_ref_copy;
        
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            current_euler = euler;
            angle_zero_ref_copy = angle_zero_ref;
            xSemaphoreGive(data_mutex);
        } else {
            ESP_LOGE(TAG, "Failed to take mutex in sensor_task");
            memset(&angle_zero_ref_copy, 0, sizeof(lsm6ds3_angle_zero_t));
        }
        
        float roll_display = euler.roll;
        float pitch_display = euler.pitch;
        float yaw_display = euler.yaw;
        float comp_roll_display = comp_roll;
        float comp_pitch_display = comp_pitch;
        
        lsm6ds3_apply_angle_zero(&angle_zero_ref_copy, &roll_display, &pitch_display, &yaw_display);
        lsm6ds3_apply_angle_zero(&angle_zero_ref_copy, &comp_roll_display, &comp_pitch_display, NULL);
        
        float ground_angle_deg = lsm6ds3_calculate_angle_from_vertical(roll_display, pitch_display);
        
        sample_count++;
        if (sample_count % 10 == 0) {
            printf("%6.1f %6.1f %6.1f | %6.2f %6.2f %6.2f | %6.2f %6.2f %6.2f | %6.2f %6.2f | %6.2f\n",
                   accel_mg[0], accel_mg[1], accel_mg[2],
                   gyro_dps[0], gyro_dps[1], gyro_dps[2],
                   roll_display, pitch_display, yaw_display,
                   comp_roll_display, comp_pitch_display,
                   ground_angle_deg);
        }
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS((uint32_t)(dt * 1000.0f)));
    }
}

static void gpio_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_ROLL_ZERO) | (1ULL << GPIO_PITCH_ZERO) | (1ULL << GPIO_YAW_ZERO),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "GPIO initialized:");
    ESP_LOGI(TAG, "  GPIO %d = Roll Zero (pull-up, active low)", GPIO_ROLL_ZERO);
    ESP_LOGI(TAG, "  GPIO %d = Pitch Zero (pull-up, active low)", GPIO_PITCH_ZERO);
    ESP_LOGI(TAG, "  GPIO %d = Yaw Zero (pull-up, active low)", GPIO_YAW_ZERO);
}

void app_main(void)
{
    ESP_LOGI(TAG, "LSM6DS3 Sensor Fusion Example");
    ESP_LOGI(TAG, "=============================");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    
    memset(&angle_zero_ref, 0, sizeof(lsm6ds3_angle_zero_t));
    ret = lsm6ds3_load_angle_zero_from_nvs(&angle_zero_ref, "lsm6ds3");
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Loaded angle zero references from NVS");
    } else {
        ESP_LOGI(TAG, "No angle zero references found in NVS");
    }
    
    gpio_init();
    
    ESP_LOGI(TAG, "Initializing I2C master...");
    ESP_ERROR_CHECK(i2c_master_init());
    
    ESP_LOGI(TAG, "Initializing LSM6DS3 sensor...");
    
    lsm6ds3_config_t sensor_config = {
        .interface = LSM6DS3_INTERFACE_I2C,
        .bus.i2c.bus_handle = i2c_bus_handle,
        .bus.i2c.address = LSM6DS3_I2C_ADDR,
    };
    
    ret = lsm6ds3_init(&sensor_handle, &sensor_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LSM6DS3: %s", esp_err_to_name(ret));
        return;
    }
    
    uint8_t device_id;
    lsm6ds3_get_device_id(&sensor_handle, &device_id);
    ESP_LOGI(TAG, "Device ID: 0x%02X", device_id);
    
    ESP_LOGI(TAG, "Resetting sensor...");
    lsm6ds3_reset(&sensor_handle);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "Configuring sensor...");
    
    lsm6ds3_enable_block_data_update(&sensor_handle, true);
    
    lsm6ds3_set_accel_odr(&sensor_handle, LSM6DS3_XL_ODR_104Hz);
    lsm6ds3_set_accel_full_scale(&sensor_handle, LSM6DS3_2g);
    
    lsm6ds3_set_gyro_odr(&sensor_handle, LSM6DS3_GY_ODR_104Hz);
    lsm6ds3_set_gyro_full_scale(&sensor_handle, LSM6DS3_2000dps);
    
    ESP_LOGI(TAG, "Sensor configured:");
    ESP_LOGI(TAG, "  Accelerometer: 104 Hz, ±2g");
    ESP_LOGI(TAG, "  Gyroscope: 104 Hz, ±2000 dps");
    ESP_LOGI(TAG, "");
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(gpio_zero_task, "gpio_zero_task", 2048, NULL, 5, NULL);
}

