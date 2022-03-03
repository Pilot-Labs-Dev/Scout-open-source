/*
 * i2c interface to access i2c-dev
 *
 * Author      : Peter Yang
 * Create Time : Dec 2018
 * Change Log  :
 *     11:09 2018/12/26 Initial version
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software", to deal
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include<errno.h>
#include<assert.h>
#include "rpi_i2c.h"

#define RAW_MAX		0x8000
#define MAX_WRITE_BUFF	16
static int rpi_i2c_fd = -1;


#define DEBUG_ERROR	0
#if DEBUG_ERROR
#define DBG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define DBG(fmt, ...)  
#endif

int rpi_i2c_init(const char* dev_path) {
	int fd;

	if ((fd = rpi_i2c_fd) >= 0) {
		return fd;
	}

	if ((fd = open(dev_path, O_RDWR)) < 0) {
		DBG("Failed to open i2c bus %s, error = %d\n",
		       dev_path, fd);
	} else {
		rpi_i2c_fd = fd;
	}
	return fd;
}

int8_t rpi_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) 
{
    int retval,i;
    uint8_t buff[MAX_WRITE_BUFF];

	assert(len+1<=MAX_WRITE_BUFF);

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    buff[0] = reg_addr;
	for(i=0;i<len;i++)
    	buff[i+1] = data[i];

    msgs[0].addr = dev_addr;
    msgs[0].flags = 0;
    msgs[0].len = len+1;
    msgs[0].buf = buff;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(rpi_i2c_fd, I2C_RDWR, &msgset) < 0) {
        DBG("12c_write ioctl failed and returned errno[%s]\n", strerror(errno));
        return RPI_I2C_FAIL;
    }

    return RPI_I2C_OK;
}

int8_t rpi_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    int retval;
    uint8_t outbuf[1];
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

	outbuf[0] = reg_addr;
    msgs[0].addr = dev_addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = outbuf;

    msgs[1].addr = dev_addr;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = len;
    msgs[1].buf = data;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;


    if (ioctl(rpi_i2c_fd, I2C_RDWR, &msgset) < 0) {
        DBG("i2c_read ioctl failed and returned errno [%s \n", strerror(errno));
        return RPI_I2C_FAIL;
    }

    return RPI_I2C_OK;
}

void rpi_delay_ms(uint32_t millis) {
	usleep(millis * 1000UL);
	return;
}

int i2c_read_byte(uint8_t dev, uint8_t reg) {
	uint8_t data;

	if (rpi_i2c_read(dev, reg, &data, 1)) {
		return RPI_I2C_FAIL;
	}
	return data;
}

int i2c_read_word(uint8_t dev, uint8_t reg) {
	uint8_t data[2];

	if (rpi_i2c_read(dev, reg, data, 2)) {
		return RPI_I2C_FAIL;
	}
	return ((unsigned)data[0] << 8) | data[1];
}

int i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t data) {
	return rpi_i2c_write(dev, reg, &data, 1);
}

int i2c_write_word(uint8_t dev, uint8_t reg, uint16_t data) {
	uint8_t buffer[2];

	buffer[0] = data >> 8;
	buffer[1] = data & 0xFF;
	if (rpi_i2c_write(dev, reg, buffer, 2)) {
		return RPI_I2C_FAIL;
	}
	return RPI_I2C_OK;
}
