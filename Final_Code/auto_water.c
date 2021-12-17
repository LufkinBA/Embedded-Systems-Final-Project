#include <stdio.h>

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "auto_water.h"
#include "linux/gpio.h"
#include "sys/ioctl.h"

#define LENGTH 3
int main(int argc, char **argv) {
	
	int gpio_fd;
	struct gpiohandle_request req; 
	float moisture_reading; 
	
	moisture_reading = read_moisture();

	printf("The moisture is %lf V\n", moisture_reading);
	if (moisture_reading > 2.2)
	{	
		gpio_fd = gpio_open();

		req = gpio_init(gpio_fd);	

		run_pump(3, gpio_fd, req);
	}
	return 0; 
}


double read_moisture()
{
	int spi_fd, adc_conv, result, lsb_mode, mode;
	float conv_num;
	unsigned char data_out[LENGTH] = {0x0,0x0,0x0};
	unsigned char data_in[LENGTH];
	struct spi_ioc_transfer spi;
	
	/* Open SPI device */
    spi_fd = open("/dev/spidev0.0", O_RDWR);

    if (spi_fd<0)
    {
        printf("Error Opening SPI Device\n");
        exit(spi_fd);
    }
   


	/* Set SPI Mode_0 */
    mode = SPI_MODE_0;
    result = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);

    //Error Checking
    if (result<0)
    {
        printf("Error communicating with SPI Device\n");
        exit(result);
    }

    /* Set SPI to LSB FIRST */
    lsb_mode = 0;
    result = ioctl(spi_fd, SPI_IOC_WR_LSB_FIRST, &lsb_mode);

    //Error Checking
    if (result<0)
    {
        printf("Error communicating with SPI Device\n");
        exit(result);
    }

    /* Set SPI transfer structure */
    memset(&spi, 0, sizeof(struct spi_ioc_transfer));

    spi.tx_buf      = (unsigned long)&data_out;
    spi.rx_buf      = (unsigned long)&data_in;
    spi.len         = LENGTH;                   //3 bytes
    spi.delay_usecs     = 0;
    spi.speed_hz        = 100000;               //100kHz
    spi.bits_per_word   = 8;
    spi.cs_change       = 0;
	 
	/* Single High read CH1. data out 0 has to be 1s, data_out[1] is set 
     * to single rather than differential 
     */
    data_out[0] = 0x1;  //0x1 as the start bit  
    data_out[1] = 0x90; //1010 0000 as channel 1 select
    data_out[2] = 0x0;  //0x0 and/or dontcares

        /* Initiate SPI transfer */
    result = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi);

        //Error Checking
    if (result<0)
    {
            printf("Error communicating with SPI Device\n");
            exit(result);
    }

    /* Get the Channel 1 data from the transfer */
	adc_conv = data_in[1] & 0x3;
    adc_conv = (adc_conv << 8) | data_in[2];

    /* convert 10 bit number to floating point number signifying 
     * the voltage on CH1
     */
    conv_num = ((float)adc_conv*(3.3))/1024;
		
	return conv_num; 
}

int gpio_open(void)
{
	int fd;
	
	    //Open GPIO device
    fd = open("/dev/gpiochip0",O_RDWR);

    //check to see if open fails    
    if (fd < 0)
    {
        printf("Failed to open GPIO file");
        exit(fd);
    }
	
	return fd;
}

struct gpiohandle_request gpio_init(int fd)
{
	int rv;
	struct gpiohandle_request req;	
    
	//Set up the request struct
    memset(&req,0,sizeof(struct gpiohandle_request));
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.lines = 1;
    req.lineoffsets[0] = 17;    //GPIO Number
    req.default_values[0] = 0;
    strcpy(req.consumer_label, "ECE471");
    rv = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);

    //rv error check
    if (rv < 0)
    {
        printf("Error ioctl %s\n", strerror(errno));
        exit(-1);
    }

	return req;
}

void run_pump(int seconds, int fd, struct gpiohandle_request req)
{
	int rv; 
	struct gpiohandle_data data; 
	
	        //LED on
        data.values[0] = 1;
        rv = ioctl(req.fd,GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        if (rv < 0)
        {
            printf("Error ioctl %s\n", strerror(errno));
            exit(-1);
        }

        //delay for .5 sec
        usleep(seconds*1000000);

        //LED off
      data.values[0] = 0;
      rv = ioctl(req.fd,GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
      if (rv < 0) 
      {
          printf("Error ioctl %s\n", strerror(errno));
          exit(-1);
      }

}
