/* auto_water.h */
#include "linux/gpio.h"
#include "sys/ioctl.h"

double read_moisture(struct spi_ioc_transfer spi,int spi_fd);

void run_pump(int seconds, int fd, struct gpiohandle_request req);

struct spi_ioc_transfer init_spi(int spi_fd);

int open_spi(void);

int gpio_open(void);

struct gpiohandle_request gpio_init(int fd);
