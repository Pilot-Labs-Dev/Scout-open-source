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
#ifndef __i2c_rpi_h__
#define __i2c_rpi_h__

#include <stdint.h>

#define RPI_I2C_OK	0
#define RPI_I2C_FAIL	-1

#ifdef __cplusplus
extern "C" {
#endif

int rpi_i2c_init(
	/* eg. /dev/i2c-1 */
	const char* dev_path
);
int8_t rpi_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t rpi_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
void rpi_delay_ms(uint32_t millis);

int i2c_read_byte(uint8_t dev, uint8_t reg);
int i2c_read_word(uint8_t dev, uint8_t reg);
int i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t data);
int i2c_write_word(uint8_t dev,uint8_t reg, uint16_t data);

#ifdef __cplusplus
}
#endif

#endif//__i2c_rpi_h__
