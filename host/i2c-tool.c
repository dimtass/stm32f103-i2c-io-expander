/*
 * Copyright (C) MIT.
 *
 * Description: This is an example of using the I2C and SPI interfaces
 * 	to read the ADC value of an SPI photoresistor and then send the value
 * 	on a I2C PWM LED. You can find more info here:
 * 	http://www.stupid-projects.com/linux-and-the-i2c-and-spi-interface/
 *
 * Author: Dimitris Tassopoulos <dimtass@gmail.com>
 *
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <errno.h>
// #include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <argp.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#define POLLING_MS	50
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define STR_MAX_SIZE 32

#define DEBUG
#ifdef DEBUG
#define TRACE(X) do { printf X ;} while(0)
#else
#define TRACE(x)
#endif

/** I2C **/
#define I2C_ADDR 0x08

struct i2c_dev {
	char 	device[STR_MAX_SIZE];
	int		fd;
	int		addr;
};

int i2c_init(struct i2c_dev * dev)
{
	if ((dev->fd = open(dev->device, O_RDWR)) < 0) {
	    /* ERROR HANDLING: you can check errno to see what went wrong */
	    perror("Failed to open the i2c bus");
	    exit(1);
	}
	return dev->fd;
}

void i2c_close(struct i2c_dev * dev)
{
	close(dev->fd);
}

int i2c_read_register(struct i2c_dev * dev, uint8_t reg, uint8_t *result)
{
	int retval;
	uint8_t outbuf[1], inbuf[1];
	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data msgset[1];

	msgs[0].addr = dev->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = outbuf;

	msgs[1].addr = dev->addr;
	msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
	msgs[1].len = 1;
	msgs[1].buf = inbuf;

	msgset[0].msgs = msgs;
	msgset[0].nmsgs = 2;

	outbuf[0] = reg;
	inbuf[0] = 0;

	*result = 0;
	if (ioctl(dev->fd, I2C_RDWR, &msgset) < 0) {
		perror("ioctl(I2C_RDWR) in i2c_read");
		return -1;
	}
	*result = inbuf[0];
	return 0;
}

int i2c_write_register(struct i2c_dev * dev, uint8_t reg, uint8_t data)
{
	int retval;
    uint8_t outbuf[2];

    struct i2c_msg msgs;
    struct i2c_rdwr_ioctl_data msgset;

    outbuf[0] = reg;
    outbuf[1] = data;

    msgs.addr = dev->addr;
    msgs.flags = 0;
    msgs.len = 2;
    msgs.buf = outbuf;

    msgset.msgs = &msgs;
    msgset.nmsgs = 1;

    if (ioctl(dev->fd, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_write");
        return -1;
    }
    return 0;
}

void print_usage(int argc, char** argv)
{
	printf(
		"Usage:\n"
		"%s [I2C DEV] [SPI DEV]\n"
		"	[I2C DEV]: the I2C device/bus that the photoresistor is connected (e.g. /dev/i2c-0)\n"
		"	[SPI DEV]: the SPI device/bus that the PWM LED is connected (e.g. /dev/spidev0.0)\n\n",
		argv[0]
	);
}

long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    // printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}

static void usage(const char *argv0)
{
    fprintf(stderr, "Usage:\n"
		"%s [-i I2C_DEV]\n"
		"\t-i   : the I2C device/bus that the dummy sensor is connected (e.g. /dev/i2c-0)\n"
		"\t-r   : the I2C register to read or write\n"
		"\t-m   : R/W mode. 0=READ, 1=WRITE\n"
		"\t-b   : byte to write when -m is set to 1 (WRITE)\n"
		,
		argv0);
    exit(EXIT_FAILURE);
}

int main(int argc, char** argv)
{
	int opt;
	uint8_t reg = 0;
	enum en_rw_mode {
		I2C_READ = 1,
		I2C_WRITE = 2
	};
	uint8_t rw_mode = 0;
	uint8_t hex_byte = 0;
	uint8_t byte = 0;

	/* Create the I2C data */
	struct i2c_dev i2c = {
		.fd = -1,
		.addr = I2C_ADDR
	};

    while ((opt = getopt(argc, argv, "i:b:m:r:b:h:")) != -1)
    {
        switch (opt)
        {
		case 'i':
			strncpy(i2c.device, optarg, STR_MAX_SIZE);
			break;
		case 'r':
			reg = (uint8_t)strtol(optarg, NULL, 16);
			break;
		case 'm':
			rw_mode = atoi(optarg);
			break;
		case 'b':
			hex_byte = (uint8_t)strtol(optarg, NULL, 16);
			break;
		case 'h':
        default:
            usage(argv[0]);
			exit(0);
        }
    }
	printf("\tI2C dev: %s\n", i2c.device);
	printf("\n");

	/* init i2c */
	i2c_init(&i2c);

	uint32_t counter = 0;
	long long start = current_timestamp();
	long long stop = 0;
	int msec = 0;
	int *benchmark_results;

	switch(rw_mode) {
	case I2C_READ:
		printf("Reading register: 0x%02X...\n", reg);
		i2c_read_register(&i2c, reg, &byte);
		printf("Result [0x%02X]=0x%02X\n", reg, byte);
		break;
	case I2C_WRITE:
		printf("Write value 0x%02X to register: 0x%02X...\n", hex_byte, reg);
		i2c_write_register(&i2c, reg, hex_byte);
		break;
	};

	/* clean up */
	i2c_close(&i2c);

    return 0;
}
