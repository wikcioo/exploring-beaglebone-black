/*
 * Simple MPU6050 driver which outputs G-force (m/s^2)
 * and rotation (deg/s) values from x, y and z.
 * Alternatively, it can output pitch, roll and yaw values in degrees
 * from the accelerometer.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/*
 * AD0 = GND: slave address 0x68
 * AD0 = VCC: slave address 0x69
 */
#define MPU6050_SLAVE_ADDR  0x68
/* #define MPU6050_SLAVE_ADDR  0x69 */

/*
 * On BeagleBone Black i2c-2 has the following connections
 * P9_20 => SDA
 * P9_19 => SCL
 */
#define MPU6050_I2C_DEVICE_FILE "/dev/i2c-2"

/* MPU6050 registers */
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_GYRO_CONFIG     0x1B

#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_GYRO_XOUT_H     0x43

/* MPU6050 bit positions */
#define MPU6050_PWR_MGMT_1_SLEEP    6
#define MPU6050_ACC_CONFIG_AFS_SEL  3
#define MPU6050_GYRO_CONFIG_FS_SEL  3

/* Full-scale ranges for accelerometer and gyroscope */
#define MPU6050_ACC_FS_SEL_0        0x00
#define MPU6050_ACC_FS_SEL_1        0x01
#define MPU6050_ACC_FS_SEL_2        0x02
#define MPU6050_ACC_FS_SEL_3        0x03

#define MPU6050_GYRO_FS_SEL_0       0x00
#define MPU6050_GYRO_FS_SEL_1       0x01
#define MPU6050_GYRO_FS_SEL_2       0x02
#define MPU6050_GYRO_FS_SEL_3       0x03

/* Sensitivity values for range 0 of accelerometer and gyroscope */
#define MPU6050_ACC_FS_SEL_0_SENS   16384
#define MPU6050_GYRO_FS_SEL_0_SENS  131

/* Other MPU6050 macros */
#define MPU6050_SUCCESS         1
#define MPU6050_FAIL            0
#define MPU6050_OPERATION_SET   1
#define MPU6050_OPERATION_CLEAR 0

/* I2C file descriptor */
int fd;

/* Function prototypes */
void    mpu6050_init        (uint8_t acc_range, uint8_t gyro_range);
uint8_t mpu6050_mem_write   (uint8_t reg, uint8_t *tx_buffer);
uint8_t mpu6050_mem_read    (uint8_t reg, uint8_t *rx_buffer, uint32_t length);
void    mpu6050_get_raw_acc (uint16_t *buffer);
void    mpu6050_get_raw_gyro(uint16_t *buffer);
void    mpu6050_update_reg  (uint8_t reg, uint8_t value, uint8_t operation);

int main(int argc, char *argv[])
{
    int16_t acc_values[3], gyro_values[3];
    double acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;

    /* Set file descriptor to point at I2C device */
    if ((fd = open(MPU6050_I2C_DEVICE_FILE, O_RDWR)) < 0) {
        fprintf(stderr, "ERROR: failed to open I2C device file at: %s\n", MPU6050_I2C_DEVICE_FILE);
        return 1;
    }

    /* Set file descriptor to point at SLAVE_ADDR */
    if (ioctl(fd, I2C_SLAVE, MPU6050_SLAVE_ADDR) < 0) {
        fprintf(stderr, "ERROR: failed to set I2C slave address\n");
        close(fd);
        return 1;
    }

    uint8_t acc_full_scale  = MPU6050_ACC_FS_SEL_0;
    uint8_t gyro_full_scale = MPU6050_GYRO_FS_SEL_0;

    double acc_divisor = MPU6050_ACC_FS_SEL_0_SENS / (acc_full_scale + 1);
    double gyro_divisor = MPU6050_GYRO_FS_SEL_0_SENS / (gyro_full_scale + 1);
    printf("acc_divisor = %f, gyro_divisor = %f\n", acc_divisor, gyro_divisor);

    mpu6050_init(acc_full_scale, gyro_full_scale);

    while (1) {
        mpu6050_get_raw_acc(acc_values);
        mpu6050_get_raw_gyro(gyro_values);

        acc_x = (double)(acc_values[0] / acc_divisor);
        acc_y = (double)(acc_values[1] / acc_divisor);
        acc_z = (double)(acc_values[2] / acc_divisor);

        gyro_x = (double)(gyro_values[0] / gyro_divisor);
        gyro_y = (double)(gyro_values[1] / gyro_divisor);
        gyro_z = (double)(gyro_values[2] / gyro_divisor);

        //printf("raw acc: x = %d, y = %d, z = %d | raw gyro: x = %d, y = %d, z = %d\n",
        //        acc_values[0], acc_values[1], acc_values[2], gyro_values[0], gyro_values[1], gyro_values[2]);

        printf("%0.2f  %0.2f  %0.2f\n", acc_x, acc_y, acc_z);

        usleep(500 * 1000);
    }
    
    return 0;
}

void mpu6050_init(uint8_t acc_range, uint8_t gyro_range)
{
    /* Exit sleep mode */
    mpu6050_update_reg(MPU6050_REG_PWR_MGMT_1, (1 << MPU6050_PWR_MGMT_1_SLEEP), MPU6050_OPERATION_CLEAR);

    /* Set full scale range for the accelerometer */
    mpu6050_update_reg(MPU6050_REG_ACCEL_CONFIG, (acc_range << MPU6050_ACC_CONFIG_AFS_SEL), MPU6050_OPERATION_SET);

    /* Set full scale range for the gyroscope */
    mpu6050_update_reg(MPU6050_REG_GYRO_CONFIG, (gyro_range << MPU6050_GYRO_CONFIG_FS_SEL), MPU6050_OPERATION_SET);
}

uint8_t mpu6050_mem_write(uint8_t reg, uint8_t *tx_buffer)
{
    int res;
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = *tx_buffer;

    res = write(fd, buf, 2);
    if (res <= 0) {
        fprintf(stderr, "ERROR: failed to write to memory\n");
        return MPU6050_FAIL;
    }

    return MPU6050_SUCCESS;
}

uint8_t mpu6050_mem_read(uint8_t reg, uint8_t *rx_buffer, uint32_t length)
{
    int res;
    res = write(fd, &reg, 1);
    if (res <= 0) {
        fprintf(stderr, "ERROR: failed to write reg address\n");
        return MPU6050_FAIL;
    }

    res = read(fd, rx_buffer, length);
    if (res <= 0) {
        fprintf(stderr, "ERROR: failed to read into rx_buffer\n");
        return MPU6050_FAIL;
    }

    return MPU6050_SUCCESS;
}

void mpu6050_get_raw_acc(uint16_t *buffer)
{
    uint8_t acc_buffer[6];

    mpu6050_mem_read(MPU6050_REG_ACCEL_XOUT_H, acc_buffer, 6);

    buffer[0] = (int16_t)((acc_buffer[0] << 8) | acc_buffer[1]);
    buffer[1] = (int16_t)((acc_buffer[2] << 8) | acc_buffer[3]);
    buffer[2] = (int16_t)((acc_buffer[4] << 8) | acc_buffer[5]);
}

void mpu6050_get_raw_gyro(uint16_t *buffer)
{
    uint8_t gyro_buffer[6];

    mpu6050_mem_read(MPU6050_REG_GYRO_XOUT_H, gyro_buffer, 6);

    buffer[0] = (int16_t)((gyro_buffer[0] << 8) | gyro_buffer[1]);
    buffer[1] = (int16_t)((gyro_buffer[2] << 8) | gyro_buffer[3]);
    buffer[2] = (int16_t)((gyro_buffer[4] << 8) | gyro_buffer[5]);
}

void mpu6050_update_reg(uint8_t reg, uint8_t value, uint8_t operation)
{
    uint8_t memory;
    mpu6050_mem_read(reg, &memory, 1);
    if (operation == MPU6050_OPERATION_SET) {
        memory |= value;
    } else {
        memory &= ~value;
    }
    mpu6050_mem_write(reg, &memory);
}
