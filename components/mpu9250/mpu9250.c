/*
 * Copyright (C)  Singularita s.r.o. <info@singularita.net>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <esp_log.h>
#include <esp_err.h>

#include <mpu9250.h>


/* Logging tag. */
static const char *tag = "mpu9250";


/* I2C address to use. */
static const uint8_t addr = 0x68;


/* I2C master port to use. */
static i2c_port_t port = -1;


void mpu9250_init(i2c_port_t _port)
{
	ESP_LOGI(tag, "Initializing MPU9250...");
	port = _port;

	/* Reset the internal registers and restore the default settings. */
	i2ce_put(port, addr, 0x6b, 0x81);

	/* Auto select the best available clock source:
	 * PLL if ready, else use the Internal oscillator.
	 */
	i2ce_set(port, addr, 0x6b, 0xfe, 0x01);

	ESP_LOGI(tag, "Enabling MPU9250 bypass mode...");

	/*
	 * Disable I2C Master I/F module;
	 * pins ES_DA and ES_SCL are logically driven by pins SDA and SCL.
	 */
	i2ce_set(port, addr, 0x6a, 0xdf, 0x00);

	/*
	 * When asserted, the i2c_master interface pins (ES_CL and ES_DA)
	 * will go into ‘bypass mode’ when the i2c master interface is
	 * disabled.
	 */
	i2ce_set(port, addr, 0x37, 0xfd, 0x02);

	/* Make sure the bypass mode is active. */
	uint8_t buf[1];
	i2ce_read(port, addr, 0x37, buf, 1);

	if (!(buf[0] & 0x02)) {
		ESP_LOGE(tag, "Failed to enable bypass mode!");
		abort();
	}
}


void mpu9250_read_raw(float accm[3], float gyro[3], float temp[1])
{
	/* Layout: accm(xx yy zz) temp(tt) gyro(xx yy zz) in big-endian. */
	uint8_t buf[14];

	i2ce_read(port, addr, 0x3b, buf, sizeof(buf));

	if (accm) {
		accm[0] = (int16_t)((buf[ 0] << 8) | buf[ 1]);
		accm[1] = (int16_t)((buf[ 2] << 8) | buf[ 3]);
		accm[2] = (int16_t)((buf[ 4] << 8) | buf[ 5]);
	}

	if (temp) {
		temp[0] = (int16_t)((buf[ 6] << 8) | buf[ 7]);
	}

	if (gyro) {
		gyro[0] = (int16_t)((buf[ 8] << 8) | buf[ 9]);
		gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);
		gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);
	}
}
