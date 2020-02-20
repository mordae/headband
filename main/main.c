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

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_pm.h>

#include <nvs_flash.h>

#include <i2ce.h>
#include <mpu9250.h>
#include <ak8963.h>
#include <spatial.h>


/* Tag for logging. */
static const char tag[] = "main";


static nvs_handle_t nvs;


struct calibration {
	float magm_min[3];
	float magm_max[3];
};

struct calibration calibration = {};


static void init_chip(void)
{
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);

	ESP_LOGI(tag, "Running on a %i-core %s r%i chip.",
			chip_info.cores,
			CONFIG_IDF_TARGET,
			chip_info.revision);

	ESP_LOGI(tag, "We have a %iMB %s flash.",
			spi_flash_get_chip_size() / (1<<20),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH)
				? "embedded" : "external");

	ESP_LOGI(tag, "Configure non-volatile storage...");

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_LOGI(tag, "Erasing NVS...");
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(nvs_open("headband", NVS_READWRITE, &nvs));
}


static void init_i2c(void)
{
	i2ce_master_init(I2C_NUM_0,
	                 CONFIG_MPU9250_SDA_GPIO,
	                 CONFIG_MPU9250_SCL_GPIO,
	                 100000);
}


static void init_sensors(void)
{
	/* First enable the accelerometer with gyroscope. */
	mpu9250_init(I2C_NUM_0);

	/* Now initialize the magnetometer. */
	ak8963_init(I2C_NUM_0);
}


static void delay(unsigned ms)
{
	static TickType_t until = 0;

	TickType_t now = xTaskGetTickCount();

	if (until < now)
		until = now;

	until += pdMS_TO_TICKS(ms);
	vTaskDelay(until - now);
}


void app_main()
{
	init_chip();
	init_i2c();
	init_sensors();

	float *magm_min = calibration.magm_min;
	float *magm_max = calibration.magm_max;

	size_t len = sizeof(calibration);

	if (ESP_OK != nvs_get_blob(nvs, "calib", &calibration, &len)) {
		ESP_LOGE(tag, "No previous calibration data found.");
	}

	if (len != sizeof(calibration)) {
		ESP_LOGE(tag, "Calibration data size changed, ignoring.");
		memset(&calibration, 0, sizeof(calibration));
	}

	for (size_t i = 1; /**/; i++) {
		float accm[3], gyro[3], tmp[3], magm[3], temp;

		mpu9250_read_raw(accm, gyro, &temp);

		if (!ak8963_read_raw(tmp)) {
			ESP_LOGE(tag, "Magnetometer overflow, skipping...");
			continue;
		}

		/*
		 * Align magnetometer with the accelerometer and gyroscope.
		 * https://github.com/kriswiner/MPU9250/pull/370#issuecomment-491806904
		 */
		magm[0] =  tmp[1];
		magm[1] =  tmp[0];
		magm[2] = -tmp[2];

		/* Scale temperature properly. */
		temp = temp / 333 + 21;

		/*
		 * Remove hard iron affects by storing minimum and maximum
		 * values and then removing the offset from center.
		 */
		magm_min[0] = fmin(magm_min[0], magm[0]);
		magm_min[1] = fmin(magm_min[1], magm[1]);
		magm_min[2] = fmin(magm_min[2], magm[2]);

		magm_max[0] = fmax(magm_max[0], magm[0]);
		magm_max[1] = fmax(magm_max[1], magm[1]);
		magm_max[2] = fmax(magm_max[2], magm[2]);

		/* Persist calibration from time to time. */
		if (0 == (i % 600)) {
			ESP_LOGI(tag, "Saving calibration data...");
			ESP_ERROR_CHECK(nvs_set_blob(nvs, "calib", &calibration, sizeof(calibration)));
		}

		magm[0] -= (magm_min[0] + magm_max[0]) / 2.0;
		magm[1] -= (magm_min[1] + magm_max[1]) / 2.0;
		magm[2] -= (magm_min[2] + magm_max[2]) / 2.0;

		/*
		 * Remove soft iron effects by calculating average radius
		 * and then scaling the individual axes accordingly.
		 */
		float vmin[3] = {
			magm_min[0] - (magm_min[0] + magm_max[0]) / 2.0,
			magm_min[1] - (magm_min[1] + magm_max[1]) / 2.0,
			magm_min[2] - (magm_min[2] + magm_max[2]) / 2.0,
		};

		float vmax[3] = {
			magm_max[0] - (magm_min[0] + magm_max[0]) / 2.0,
			magm_max[1] - (magm_min[1] + magm_max[1]) / 2.0,
			magm_max[2] - (magm_min[2] + magm_max[2]) / 2.0,
		};

		float avgs[3] = {
			(vmax[0] - vmin[0]) / 2.0,
			(vmax[1] - vmin[1]) / 2.0,
			(vmax[2] - vmin[2]) / 2.0,
		};

		float avg_radius = (avgs[0] + avgs[1] + avgs[2]) / 3.0;

		magm[0] *= avg_radius / avgs[0];
		magm[1] *= avg_radius / avgs[1];
		magm[2] *= avg_radius / avgs[2];

#if 0
		printf("%6.0f %6.0f %6.0f / %6.0f %6.0f %6.0f / %6.0f %6.0f %6.0f / %4.0f°C\n",
		       accm[0], accm[1], accm[2],
		       gyro[0], gyro[1], gyro[2],
		       magm[0], magm[1], magm[2],
		       temp);
#endif

#if 1
		vec3 down  = {{accm[0], accm[1], accm[2]}};
		vec3 east  = {{magm[0], magm[1], magm[2]}};
		east = vec3cross(down, east);
		vec3 north = vec3cross(east, down);

		down  = vec3unit(down);
		east  = vec3unit(east);
		north = vec3unit(north);

		mat3 rm = {{north, east, down}};
		quat q = quat_from_mat3(rm);
		vec3 euler = quat_to_euler(q);

		printf("QTR: [%f, %f, %f, %f]\n",
		       q.w, q.x, q.y, q.z);

		printf("RPY: [%f, %f, %f]\n",
		       euler.row[0] * 180 / M_PI,
		       euler.row[1] * 180 / M_PI,
		       euler.row[2] * 180 / M_PI);
#endif

		delay(10);
	}
}
