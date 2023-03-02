#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "gpio_driver.h"

void gpio_export(uint8_t gpio_number)
{
    char path[BUFFER_SIZE];
    snprintf(path, sizeof(path), SYSFS_GPIO_PATH "/export");

    int fd = open(path, O_WRONLY);
    if (fd < 0)
    {
        fprintf(stderr, "Error writing to a file: %s, Description: %s\n", path, strerror(errno));
        return;
    }

    char buf[BUFFER_SIZE];
    size_t length = snprintf(buf, sizeof(buf), "%d", gpio_number);
    ssize_t bytes_written = write(fd, buf, length);
    if (bytes_written <= 0)
    {
        fprintf(stderr, "Error writing to a file: %s, Description: %s\n", path, strerror(errno));
        return;
    }

    close(fd);
}

void gpio_unexport(uint8_t gpio_number)
{
    char path[BUFFER_SIZE];
    snprintf(path, sizeof(path), SYSFS_GPIO_PATH "/unexport");

    int fd = open(path, O_WRONLY);
    if (fd < 0)
    {
        fprintf(stderr, "Error writing to a file: %s, Description: %s\n", path, strerror(errno));
        return;
    }

    char buf[BUFFER_SIZE];
    size_t length = snprintf(buf, sizeof(buf), "%d", gpio_number);
    ssize_t bytes_written = write(fd, buf, length);
    if (bytes_written <= 0)
    {
        fprintf(stderr, "Error writing to a file: %s, Description: %s\n", path, strerror(errno));
        return;
    }

    close(fd);
}

void gpio_set_direction(uint8_t gpio_number, uint8_t direction)
{
    char path[BUFFER_SIZE];
    snprintf(path, sizeof(path), SYSFS_GPIO_PATH "/gpio%d/direction", gpio_number);

    int fd = open(path, O_WRONLY);
    if (fd < 0)
    {
        fprintf(stderr, "Error writing to a file: %s, Description: %s\n", path, strerror(errno));
        return;
    }

    ssize_t bytes_written;
    if (direction == GPIO_DIR_INPUT)
    {
        bytes_written = write(fd, "in", 3);
    }
    else
    {
        bytes_written = write(fd, "out", 4);
    }

    if (bytes_written <= 0)
    {
        fprintf(stderr, "Error writing to a file: %s, Description: %s\n", path, strerror(errno));
        return;
    }

    close(fd);
}

void gpio_write(uint8_t gpio_number, uint8_t value)
{
    char path[BUFFER_SIZE];
    snprintf(path, sizeof(path), SYSFS_GPIO_PATH "/gpio%d/value", gpio_number);

    int fd = open(path, O_WRONLY);
    if (fd < 0)
    {
        fprintf(stderr, "Error opening a file: %s, Description: %s\n", path, strerror(errno));
        return;
    }

    ssize_t bytes_written;
    if (value == GPIO_OUTPUT_HIGH)
    {
        bytes_written = write(fd, "1", 2);
    }
    else
    {
        bytes_written = write(fd, "0", 2);
    }

    if (bytes_written <= 0)
    {
        fprintf(stderr, "Error writing to a file: %s, Description: %s\n", path, strerror(errno));
        return;
    }

    close(fd);
}

void gpio_read(uint8_t gpio_number, uint8_t *rcv_buffer)
{
    char path[BUFFER_SIZE];
    snprintf(path, sizeof(path), SYSFS_GPIO_PATH "/gpio%d/value", gpio_number);

    int fd = open(path, O_RDONLY);
    if (fd < 0)
    {
        fprintf(stderr, "Error opening a file: %s, Description: %s\n", path, strerror(errno));
        return;
    }

    ssize_t bytes_read = read(fd, rcv_buffer, 1);
    if (bytes_read <= 0)
    {
        fprintf(stderr, "Error reading from a file: %s, Description: %s\n", path, strerror(errno));
        return;
    }

    close(fd);
}
