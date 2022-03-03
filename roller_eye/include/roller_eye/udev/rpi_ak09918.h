/*	
 * AK09918 device driver
 *
 * log: 11:09 2018/12/26 Initial version
 *      Base on https://github.com/Seeed-Studio/Seeed_ICM20600_AK09918.git
 *
 * The MIT License (MIT)
 *
 * Copyright (C) 2018  Peter Yang <turmary@126.com>
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
#ifndef __RPI_AK09918_H__
#define __RPI_AK09918_H__


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AK09918_I2C_ADDR	0x0C	// I2C address (Can't be changed)

// #define AK09918_MEASURE_PERIOD 9	// Must not be changed
// AK09918 has following seven operation modes:
// (1) Power-down mode: AK09918 doesn't measure
// (2) Single measurement mode: measure when you call any getData() function
// (3) Continuous measurement mode 1: 10Hz,  measure 10 times per second,
// (4) Continuous measurement mode 2: 20Hz,  measure 20 times per second,
// (5) Continuous measurement mode 3: 50Hz,  measure 50 times per second,
// (6) Continuous measurement mode 4: 100Hz, measure 100 times per second,
// (7) Self-test mode
typedef enum {
	AK09918_POWER_DOWN       = 0x00,
	AK09918_NORMAL           = 0x01,
	AK09918_CONTINUOUS_10HZ  = 0x02,
	AK09918_CONTINUOUS_20HZ  = 0x04,
	AK09918_CONTINUOUS_50HZ  = 0x06,
	AK09918_CONTINUOUS_100HZ = 0x08,
	AK09918_SELF_TEST        = 0x10,
} AK09918_mode_type_t;

typedef enum {
	AK09918_ERR_OK              = 0, // OK
	AK09918_ERR_DOR,                 // data skipped
	AK09918_ERR_NOT_RDY,             // not ready
	AK09918_ERR_TIMEOUT,             // read/write timeout
	AK09918_ERR_SELFTEST_FAILED,     // self test failed
	AK09918_ERR_OVERFLOW,            // sensor overflow, means |x|+|y|+|z| >= 4912uT
	AK09918_ERR_WRITE_FAILED,        // fail to write
	AK09918_ERR_READ_FAILED,         // fail to read
	AK09918_ERR_UNKNOWN,             // unknown error
} AK09918_err_type_t;

typedef struct {
	uint8_t addr;
	uint8_t mode;
} rpi_ak09918_t;

void* rpi_ak09918_alloc(void);
int rpi_ak09918_free(rpi_ak09918_t* dev);

// return >=0: device ID
//         <0: error  #
int rpi_ak09918_init(
	rpi_ak09918_t* dev,
	const char* i2c_dev,
	int i2c_addr,
	int mode
);

// get the working mode of AK09918
int rpi_ak09918_get_mode(rpi_ak09918_t* dev);

// set the working mode for AK09918
int rpi_ak09918_set_mode(
	rpi_ak09918_t* dev,
	AK09918_mode_type_t mode
);

// Get details of AK09918_err_type_t
const char* rpi_ak09918_err_string(/*AK09918_err_type_t*/int err);

// Reset AK09918
int rpi_ak09918_reset(rpi_ak09918_t* dev);

// At AK09918_CONTINUOUS_** mode, check if data is ready to read
int rpi_ak09918_is_ready(rpi_ak09918_t* dev);

// At AK09918_CONTINUOUS_** mode, check if data is skipped
int rpi_ak09918_is_skip(rpi_ak09918_t* dev);

// Get magnet data in uT
int rpi_ak09918_read(
	rpi_ak09918_t* dev,
	double* x, double* y, double* z
);

// Get raw I2C magnet data
int rpi_ak09918_read_raw(
	rpi_ak09918_t* dev,
	int32_t *rx, int32_t *ry, int32_t *rz
);

// Start a self-test, if pass, return AK09918_ERR_OK
int rpi_ak09918_self_test(rpi_ak09918_t* dev);

#ifdef __cplusplus
}
#endif

#endif//__RPI_AK09918_H__
