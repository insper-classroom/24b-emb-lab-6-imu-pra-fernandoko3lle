#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include <Fusion.h>

#define SAMPLE_PERIOD (0.01f)

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

QueueHandle_t xQueueI2C;

typedef struct data {
    int x;
    int y;
    int velx;
} data_t;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    data_t data;

    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    while(1) {
        //const FusionVector gyroscope = {0.0f, 0.0f, 0.0f}; // replace this with actual gyroscope data in degrees/s
        //const FusionVector accelerometer = {0.0f, 0.0f, 1.0f}; // replace this with actual accelerometer data in g
        //FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        }; 

        data.x = (int) gyroscope.axis.y;
        data.y = (int) gyroscope.axis.x;
        data.velx = (int) accelerometer.axis.x;
        
        xQueueSend(xQueueI2C, &data, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(10));
    }


}

void uart_task(void *p) {
    data_t data;
    
    while (1) {
        uint8_t pacote_x[5];
        if (xQueueReceive(xQueueI2C, &data, portMAX_DELAY)){

            uint8_t AXIS_X = 0;
            uint8_t VAL_1 = (data.x >> 8) & 0xFF;  // Byte mais significativo (MSB)
            uint8_t VAL_0 = data.x & 0xFF; // Byte menos significativo (LSB)
            uint8_t VEL = data.velx;
            uint8_t EOP = -1 & 0xFF;
            pacote_x[0] = AXIS_X;
            pacote_x[1] = VAL_1;
            pacote_x[2] = VAL_0;
            pacote_x[3] = VEL;
            pacote_x[4] = EOP;
            uart_write_blocking(uart0, pacote_x, 5);

            uint8_t pacote_y[5];
            uint8_t AXIS_Y = 1;
            uint8_t VAL_1x = (data.y >> 8) & 0xFF;  // Byte mais significativo (MSB)
            uint8_t VAL_0x = data.y & 0xFF;         // Byte menos significativo (LSB)
            pacote_y[0] = AXIS_Y;
            pacote_y[1] = VAL_1x;
            pacote_y[2] = VAL_0x;
            pacote_y[3] = data.velx;
            pacote_y[4] = EOP;
            uart_write_blocking(uart0, pacote_y, 5);
            vTaskDelay(pdMS_TO_TICKS(50));

        }
    }
}

void uart_setup() {
    // Inicializar a UART com a velocidade de transmissão (baud rate)
    uart_init(UART_ID, BAUD_RATE);

    // Configurar os pinos para a UART
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Configurar o formato de dados: 8 bits de dados, 1 bit de parada, sem paridade
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);

    // Configurar a FIFO (First In, First Out) para receber dados da UART
    uart_set_fifo_enabled(UART_ID, true);
}

int main() {
    stdio_init_all();
    uart_setup();

    xQueueI2C = xQueueCreate(32, sizeof(data_t));

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart task", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
