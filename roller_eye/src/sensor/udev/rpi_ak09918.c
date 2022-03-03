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
#include <malloc.h>
#include <unistd.h>
#include "rpi_ak09918.h"
#include "rpi_i2c.h"

/***************************************************************
 AK09918 I2C Register
 ***************************************************************/
#define AK09918_WIA1        0x00    // Company ID
#define AK09918_WIA2        0x01    // Device ID
#define AK09918_RSV1        0x02    // Reserved 1
#define AK09918_RSV2        0x03    // Reserved 2
#define AK09918_ST1         0x10    // DataStatus 1
#define AK09918_HXL         0x11    // X-axis data 
#define AK09918_HXH         0x12
#define AK09918_HYL         0x13    // Y-axis data
#define AK09918_HYH         0x14
#define AK09918_HZL         0x15    // Z-axis data
#define AK09918_HZH         0x16
#define AK09918_TMPS        0x17    // Dummy
#define AK09918_ST2         0x18    // Datastatus 2
#define AK09918_CNTL1       0x30    // Dummy
#define AK09918_CNTL2       0x31    // Control settings
#define AK09918_CNTL3       0x32    // Control settings

#define AK09918_SRST_BIT    0x01    // Soft Reset
#define AK09918_HOFL_BIT    0x08    // Sensor Over Flow
#define AK09918_DOR_BIT     0x02    // Data Over Run
#define AK09918_DRDY_BIT    0x01    // Data Ready

static const char* ak09918_err_strings[] = {
	"AK09918_ERR_OK: OK",
	"AK09918_ERR_DOR: Data skipped(read too slowly)",
	"AK09918_ERR_NOT_RDY: Not ready(check too quickly)",
	"AK09918_ERR_TIMEOUT: Timeout",
	"AK09918_ERR_SELFTEST: Self test failed",
	"AK09918_ERR_OVERFLOW: Sensor overflow",
	"AK09918_ERR_WR_FAIL: Fail to write",
	"AK09918_ERR_RD_FAIL: Fail to read",
	"Unknown Error",
};

const char* rpi_ak09918_err_string(int err) {
	const int size = sizeof ak09918_err_strings /
	                  sizeof ak09918_err_strings[0];

	err = (err < 0)? -err: err;
	if (err >= size) {
		err = size - 1;
	}
	return ak09918_err_strings[err];
}

void* rpi_ak09918_alloc(void) {
	return malloc(sizeof(rpi_ak09918_t));
}

int rpi_ak09918_free(rpi_ak09918_t* dev) {
	free(dev);
	return 0;
}

int rpi_ak09918_init(rpi_ak09918_t* dev,
	const char* i2c_dev,
	int i2c_addr,
	int mode
) {
	int rt;

	dev->addr = i2c_addr;
	dev->mode = mode;

	if ((rt = rpi_i2c_init(i2c_dev)) < 0) {
		return -AK09918_ERR_READ_FAILED;
	}

	rpi_ak09918_set_mode(dev, dev->mode);

	rt = i2c_read_word(dev->addr, AK09918_WIA1);
	return rt;
}

int rpi_ak09918_get_mode(rpi_ak09918_t* dev) {
	return dev->mode;
}

int rpi_ak09918_set_mode(
	rpi_ak09918_t* dev,
	AK09918_mode_type_t mode
) {
	if (i2c_write_byte(dev->addr, AK09918_CNTL2, mode)) {
		return AK09918_ERR_WRITE_FAILED;
	}
	dev->mode = mode;
	return AK09918_ERR_OK;
}

int rpi_ak09918_reset(rpi_ak09918_t* dev) {
	int r;

	r = i2c_write_byte(dev->addr, AK09918_CNTL3, AK09918_SRST_BIT);
	if (r < 0) {
		return AK09918_ERR_WRITE_FAILED;
	}
	return AK09918_ERR_OK;
}

int rpi_ak09918_is_ready(rpi_ak09918_t* dev) {
	int reg;

	reg = i2c_read_byte(dev->addr, AK09918_ST1);
	if (reg < 0) {
		return AK09918_ERR_READ_FAILED;
	}
        if (reg & AK09918_DRDY_BIT) {
		return AK09918_ERR_OK;
	}
        return AK09918_ERR_NOT_RDY;
}

int rpi_ak09918_is_skip(rpi_ak09918_t* dev) {
	int reg;

	reg = i2c_read_byte(dev->addr, AK09918_ST1);
	if (reg < 0) {
		return AK09918_ERR_READ_FAILED;
	}
        if (reg & AK09918_DOR_BIT) {
		return AK09918_ERR_DOR;
	}
        return AK09918_ERR_OK;
}

int rpi_ak09918_read_raw(
	rpi_ak09918_t* dev,
	int32_t *rx, int32_t *ry, int32_t *rz
) {
	uint8_t buf[8];
	int rt;

	if (dev->mode == AK09918_NORMAL) {
		int count = 0;

		for (;;) {
			rt = i2c_read_byte(dev->addr, AK09918_CNTL2);
			if (rt == 0) {
				break;
			}
			if (count++ >= 15) {
				return AK09918_ERR_TIMEOUT;
			}
			rpi_delay_ms(1);
		}
	}

	rt = rpi_i2c_read(dev->addr, AK09918_HXL, buf, sizeof buf);
	if (rt < 0) {
		return AK09918_ERR_READ_FAILED;
	}
	*rx = *(int16_t*)&buf[0];
	*ry = *(int16_t*)&buf[2];
	*rz = *(int16_t*)&buf[4];
	if (buf[7] & AK09918_HOFL_BIT) {
		return AK09918_ERR_OVERFLOW;
	}
	return AK09918_ERR_OK;
}

int rpi_ak09918_read(
	rpi_ak09918_t* dev,
	double* x, double* y, double* z
) {
	#define COEF	0.15
	int rt;
	int32_t tx, ty, tz;

	rt = rpi_ak09918_read_raw(dev, &tx, &ty, &tz);
	*x = tx * COEF;
	*y = ty * COEF;
	*z = tz * COEF;
	return rt;
}

// 1. Set Power-down mode. (MODE[4:0] bits = “00000”)
// 2. Set Self-test mode.  (MODE[4:0] bits = “10000”)
// 3. Check Data Ready or not by polling DRDY bit of ST1 register.
// 4. When Data Ready, proceed to the next step.
//    Read measurement data. (HXL to HZH)
int rpi_ak09918_self_test(rpi_ak09918_t* dev) {
	uint8_t buf[8];
	int32_t x, y, z;
	int rt, l_mode;

	l_mode = dev->mode;

	/* skip the buffer data or else self testing will failed */
	rpi_ak09918_read_raw(dev, &x, &y, &z);

	rt = rpi_ak09918_set_mode(dev, AK09918_POWER_DOWN);
	rpi_delay_ms(1);
	rt = rpi_ak09918_set_mode(dev, AK09918_SELF_TEST);

	for (;;) {
		if ((rt = rpi_ak09918_is_ready(dev)) == AK09918_ERR_OK) {
			break;
		}
		if (rt == AK09918_ERR_READ_FAILED) {
			rpi_ak09918_set_mode(dev, l_mode);
			return rt;
		}
		rpi_delay_ms(1);
	}

	rt = rpi_i2c_read(dev->addr, AK09918_HXL, buf, sizeof buf);
	if (rt < 0) {
		rpi_ak09918_set_mode(dev, l_mode);
		return AK09918_ERR_READ_FAILED;
	}
	x = *(int16_t*)&buf[0];
	y = *(int16_t*)&buf[2];
	z = *(int16_t*)&buf[4];

	if (( -200 <= x && x <= 200 ) &&
	    ( -200 <= y && y <= 200 ) &&
	    (-1000 <= z && z <= -150)
	) {
		rt = AK09918_ERR_OK;
	} else {
		rt = AK09918_ERR_SELFTEST_FAILED;
	}

	rpi_ak09918_set_mode(dev, l_mode);
	return rt;
}

