/*
 * Copyright (C)  Jan Hamal Dvořák <mordae@anilinux.org>
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

#ifndef _COMPONENT_I2CE_H
#define _COMPONENT_I2CE_H 1

#include <stdlib.h>

#include <driver/i2c.h>


/*
 * I2C Easy
 * ========
 *
 * Wraps ESP-IDF <driver/i2c.h> in a less verbose interface geared
 * towards typical I2C usage with mandatory ACKs and command codes.
 */

/* Initialize the I2C master. */
void i2ce_master_init(i2c_port_t port,
                      uint8_t sda, uint8_t scl,
                      uint32_t freq);

/* First send the command, then write the payload. */
void i2ce_write(i2c_port_t port,
                uint8_t addr, uint8_t cmd,
                const void *src, size_t len);

/* Same as i2ce_write, but send just a single byte. */
void i2ce_put(i2c_port_t port,
              uint8_t addr, uint8_t cmd,
              uint8_t value);

/* First send the command, then read the reply. */
void i2ce_read(i2c_port_t port,
               uint8_t addr, uint8_t cmd,
               void *dst, size_t len);

/*
 * First read the byte using i2ce_read(), then apply the mask,
 * then set the bits and then write the result back.
 */
void i2ce_set(i2c_port_t port,
              uint8_t addr, uint8_t cmd,
              uint8_t mask, uint8_t bits);


#endif				/* !_COMPONENT_I2CE_H */
