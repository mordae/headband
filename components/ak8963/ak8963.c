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

#include <ak8963.h>


/* Logging tag. */
static const char *tag = "ak8963";


/* I2C address to use. */
static const uint8_t addr = 0x0c;


/* I2C master port to use. */
static i2c_port_t port = -1;


/* Sensitivity adjustment data from the device. */
static float asa[3] = {0, 0, 0};


void ak8963_init(i2c_port_t _port)
{
	uint8_t buf[3];

	ESP_LOGI(tag, "Initializing AK8963...");
	port = _port;

	/* Make sure we have reached AK8963. */
	i2ce_read(port, addr, 0x00, buf, 1);

	if (buf[0] != 0x48) {
		ESP_LOGE(tag, "AK8963 WAI mismatch: %#hhx != 0x48", buf[0]);
		abort();
	}

	/*
	 * Sensitivity adjustment data for each axis is stored to fuse ROM
	 * on shipment.  We need to enter the FUSE-access mode to read them.
	 */
	i2ce_put(port, addr, 0x0a, 0x0f);

	/*
	 * When user wants to change operation mode, transit to power-down
	 * mode first and then transit to other modes. After power-down mode
	 * is set, at least 100μs is needed before setting another mode.
	 */
	vTaskDelay(pdMS_TO_TICKS(1));

	/* Now read the sensitivity adjustments. */
	i2ce_read(port, addr, 0x10, buf, 3);

	asa[0] = buf[0] - 128;
	asa[1] = buf[1] - 128;
	asa[2] = buf[2] - 128;

	/* Power down. */
	i2ce_put(port, addr, 0x0a, 0x00);
	vTaskDelay(pdMS_TO_TICKS(1));

	/*
	 * Now move onto the continuous measurement mode with
	 * 16-bit resolution.
	 */
	i2ce_put(port, addr, 0x0a, 0x16);
	vTaskDelay(pdMS_TO_TICKS(1));
}


bool ak8963_read_raw(float magm[3])
{
	/* Layout: magm(xx yy zz) status(s) in little-endian. */
	uint8_t buf[7];

	i2ce_read(port, addr, 0x03, buf, 7);

	/* First, read in the raw values. */
	magm[0] = (int16_t)((buf[1] << 8) | buf[0]);
	magm[1] = (int16_t)((buf[3] << 8) | buf[2]);
	magm[2] = (int16_t)((buf[5] << 8) | buf[4]);

	/* Then adjust them using ASA values. */
	magm[0] += (magm[0] * asa[0]) / 256.0;
	magm[1] += (magm[1] * asa[1]) / 256.0;
	magm[2] += (magm[2] * asa[2]) / 256.0;

	/*
	 * Magnetic sensor may overflow even though measurement data
	 * register is not saturated. In this case, measurement data
	 * is not correct and HOFL bit turns to 1.
	 */
	return !(buf[6] & 0x08);
}
