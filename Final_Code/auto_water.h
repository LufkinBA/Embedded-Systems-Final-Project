/* auto_water.h */
#include "linux/gpio.h"
#include "sys/ioctl.h"

double read_moisture(void);

void run_pump(int seconds, int fd, struct gpiohandle_request req);

int gpio_open(void);

struct gpiohandle_request gpio_init(int fd);
