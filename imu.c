#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"


static const char *TAG = "mpu6050";
#define I2C_ADDRESS 0x86
#define I2C_MASTER_SCL_IO 22               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21               /*!< gpio number for I2C master data  */

#define I2C_MASTER_FREQ_HZ 100000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define SLAVE_ADDRESS 0x0A

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

// Register Map
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
#define MPU6050_GYRO_CONFIG         0x1Bu
#define MPU6050_ACCEL_CONFIG        0x1Cu
#define MPU6050_INTR_PIN_CFG         0x37u
#define MPU6050_INTR_ENABLE          0x38u
#define MPU6050_INTR_STATUS          0x3Au
#define MPU6050_ACCEL_XOUT_H        0x3Bu
#define MPU6050_GYRO_XOUT_H         0x43u
#define MPU6050_TEMP_XOUT_H         0x41u
#define MPU6050_PWR_MGMT_1          0x6Bu
#define MPU6050_WHO_AM_I            0x75u

#define MPU6050_I2C_ADDRESS 0x68  // Change this if needed

void app_main(void)
{
    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,  // Define your SDA pin
        .scl_io_num = I2C_MASTER_SCL_IO,  // Define your SCL pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

    // Create MPU6050 handle
    mpu6050_handle_t sensor = mpu6050_create(I2C_NUM_0, MPU6050_I2C_ADDRESS);
    ESP_ERROR_CHECK(mpu6050_wake_up(sensor));

    while (1)
    {
        // Read accelerometer and gyro values
        mpu6050_acce_value_t acce_value;
        mpu6050_gyro_value_t gyro_value;

        esp_err_t ret = mpu6050_get_acce(sensor, &acce_value);
        if (ret != ESP_OK) {
            ESP_LOGE("MPU6050", "Failed to read accelerometer data");
            continue;
        }

        ret = mpu6050_get_gyro(sensor, &gyro_value);
        if (ret != ESP_OK) {
            ESP_LOGE("MPU6050", "Failed to read gyroscope data");
            continue;
        }

        // Print values
        printf("Accelerometer - X: %.2f, Y: %.2f, Z: %.2f\n",
               acce_value.acce_x, acce_value.acce_y, acce_value.acce_z);
        printf("Gyroscope - X: %.2f, Y: %.2f, Z: %.2f\n",
               gyro_value.gyro_x, gyro_value.gyro_y, gyro_value.gyro_z);

        // Delay before next read
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
    }

    // Cleanup
    mpu6050_delete(sensor);
}
