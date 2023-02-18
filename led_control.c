#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

#define SYSFS_LEDS_PATH "/sys/class/leds"
#define SYSFS_USR_LED_DIR "beaglebone:green:usr"

void print_usage(const char *program_name)
{
    printf("Usage: %s <led_number> <option> <value>\n", program_name);
    printf("\t<led_number> options: 0, 1, 2, 3\n");
    printf("\t<option> options: trigger, brightness\n");
    printf("\t<value> options when <option> is trigger: none, heartbeat, default-on\n");
    printf("\t<value> options when <option> is brightness: 0, 1\n");
}

int write_value_to_file(const char *path, const char *value)
{
    int fd = open(path, O_WRONLY);
    if (fd <= 0) {
        fprintf(stderr, "Failed to open %s for writing.\n", path);
        return -1;
    }

    int ret = write(fd, value, strlen(value));
    if (ret <= 0) {
        fprintf(stderr, "Failed to write to %s\n.", path);
        return -1;
    }

    return 0;
}

void process_cmd_args(char *argv[])
{
    char buf[100];
    snprintf(buf, sizeof(buf), "%s/%s%s/%s", SYSFS_LEDS_PATH, SYSFS_USR_LED_DIR, argv[1], argv[2]);

    write_value_to_file(buf, argv[3]);
}

int main(int argc, char *argv[])
{
    if (argc != 4) {
        print_usage(argv[0]);
        return 2;
    }

    process_cmd_args(argv);

    return 0;
}
