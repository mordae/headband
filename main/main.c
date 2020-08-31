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

#include <i2ce.h>
#include <mpu9250.h>
#include <ak8963.h>
#include <spatial.h>


/* Tag for logging. */
static const char tag[] = "main";


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
	init_i2c();
	init_sensors();

	float magm_off[3] = { 470.70,  342.86,  233.05};
	float magm_neg[3] = {0.94, 1.04, 0.91};
	float magm_pos[3] = {0.94, 1.02, 1.20};

	for (size_t i = 1; /**/; i++) {
		float accm[3], gyro[3], tmp[3], magm[3], temp;

		mpu9250_read_raw(accm, gyro, &temp);

		if (!ak8963_read_raw(tmp)) {
			ESP_LOGE(tag, "Magnetometer overflow, skipping...");
			continue;
		}

		/* Scale temperature properly. */
		temp = temp / 333 + 21;

		/*
		 * Align magnetometer with the accelerometer and gyroscope.
		 * https://github.com/kriswiner/MPU9250/pull/370#issuecomment-491806904
		 */
		magm[0] =  tmp[1];
		magm[1] =  tmp[0];
		magm[2] = -tmp[2];

		/*
		 * Remove hard iron effects by subtracting the offsets.
		 */
		magm[0] -= magm_off[0];
		magm[1] -= magm_off[1];
		magm[2] -= magm_off[2];

		/*
		 * Remove soft iron effects by scaling individual axes.
		 */
		magm[0] *= magm[0] < 0 ? magm_neg[0] : magm_pos[0];
		magm[1] *= magm[1] < 0 ? magm_neg[1] : magm_pos[1];
		magm[2] *= magm[2] < 0 ? magm_neg[2] : magm_pos[2];

#if 0
		printf("%6.0f %6.0f %6.0f / %6.0f %6.0f %6.0f / %6.0f %6.0f %6.0f / %4.0f°C\n",
		       accm[0], accm[1], accm[2],
		       gyro[0], gyro[1], gyro[2],
		       magm[0], magm[1], magm[2],
		       temp);
#endif

#if 1
		printf("MAG: [%f, %f, %f]\n", magm[0], magm[1], magm[2]);

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
