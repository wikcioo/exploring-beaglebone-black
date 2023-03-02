#ifndef __GPIO_DRIVER_H__
#define __GPIO_DRIVER_H__

#include <stdint.h>

#define GPIO_OUTPUT_HIGH 1
#define GPIO_OUTPUT_LOW  0

#define GPIO_DIR_INPUT  1
#define GPIO_DIR_OUTPUT 0

#define SYSFS_GPIO_PATH "/sys/class/gpio"
#define BUFFER_SIZE 64

/*
 * @param: gpio_number => value between 0 and 128
 */
void gpio_export(uint8_t gpio_number);

/*
 * @param: gpio_number => value between 0 and 128
 */
void gpio_unexport(uint8_t gpio_number);

/*
 * @param: gpio_number => value between 0 and 128
 * @param: direction   => GPIO_DIR_INPUT or GPIO_DIR_OUTPUT
 */
void gpio_set_direction(uint8_t gpio_number, uint8_t direction);

/*
 * @param: gpio_number => value between 0 and 128
 * @param: value       => GPIO_OUTPUT_HIGH or GPIO_OUTPUT_LOW
 */
void gpio_write(uint8_t gpio_number, uint8_t value);

/*
 * @param: gpio_number => value between 0 and 128
 * @param: ret_value   => pointer to the receive buffer
 */
void gpio_read(uint8_t gpio_number, uint8_t *rcv_buffer);

#endif
