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

#include <i2ce.h>


static const char *tag = "i2ce";


void i2ce_master_init(i2c_port_t port,
                      uint8_t sda, uint8_t scl,
                      uint32_t freq)
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = sda,
		.scl_io_num = scl,
		.master = {
			.clk_speed = freq,
		},
	};

	ESP_LOGI(tag, "Initializing I2C master %i...", (int)port);
	ESP_ERROR_CHECK(i2c_param_config(port, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(port, conf.mode, 0, 0, 0));
}


void i2ce_write(i2c_port_t port,
                uint8_t addr, uint8_t cmd,
                const void *src, size_t len)
{
	i2c_cmd_handle_t buf = i2c_cmd_link_create();
	i2c_master_start(buf);

	i2c_master_write_byte(buf, (addr << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(buf, cmd, 1);
	i2c_master_write(buf, (uint8_t *)src, len, 1);
	i2c_master_stop(buf);

	ESP_ERROR_CHECK(i2c_master_cmd_begin(port, buf, pdMS_TO_TICKS(1000)));

	i2c_cmd_link_delete(buf);
}


void i2ce_put(i2c_port_t port, uint8_t addr, uint8_t cmd, uint8_t value)
{
	i2ce_write(port, addr, cmd, &value, 1);
}


void i2ce_read(i2c_port_t port,
               uint8_t addr, uint8_t cmd,
               void *dst, size_t len)
{
	i2c_cmd_handle_t buf;

	/* First activate the command. */
	buf = i2c_cmd_link_create();
	i2c_master_start(buf);

	i2c_master_write_byte(buf, (addr << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(buf, cmd, 1);
	i2c_master_stop(buf);

	ESP_ERROR_CHECK(i2c_master_cmd_begin(port, buf, pdMS_TO_TICKS(1000)));

	i2c_cmd_link_delete(buf);

	/* Then read the reply. */
	buf = i2c_cmd_link_create();
	i2c_master_start(buf);
	i2c_master_write_byte(buf, (addr << 1) | I2C_MASTER_READ, 1);
	i2c_master_read(buf, dst, len, I2C_MASTER_LAST_NACK);
	i2c_master_stop(buf);

	ESP_ERROR_CHECK(i2c_master_cmd_begin(port, buf, pdMS_TO_TICKS(1000)));

	i2c_cmd_link_delete(buf);
}

void i2ce_set(i2c_port_t port,
              uint8_t addr, uint8_t cmd,
              uint8_t mask, uint8_t bits)
{
	uint8_t buf[1];

	i2ce_read(port, addr, cmd, &buf, 1);

	buf[0] = (buf[0] & mask) | bits;

	i2ce_write(port, addr, cmd, buf, 1);
}
