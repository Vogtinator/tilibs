/* Hey EMACS -*- linux-c -*- */
/* $Id$ */

/*  libticables2 - link cable library, a part of the TiLP project
 *  Copyright (C) 1999-2005  Romain Lievin
 *  Copyright (C) 2022       Fabian Vogt
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/* Cable using Pins 16 (GPIO 23) and 18 (GPIO 24) of Raspberry Pi SBCs.
 *
 * Both GPIOs can be configured as input (tristate/Hi-Z) and output (3V3/0V).
 * When the GPIO is set as input, the line voltage is not affected and it can
 * be sensed whether it is pulled low by the other side. To pull the line low,
 * it is configured as 0V output. It is never set as high (3V3) output. */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>

#include "ticables.h"
#include "error.h"
#include "logging.h"
#include "gettext.h"
#include "internal.h"

enum {
	BIT_RED = 0b01,
	BIT_WHITE = 0b10,
};

/* Syscalls are costly, so it would be beneficial to query and set both GPIO
 * lines at once. With gpiod_line_bulk, getting the values of both lines is
 * possible, but it doesn't support changing the direction of them
 * independently. This should be possible with the GPIO_V2 IOCTL API in the
 * future and libgpiod 2.x might expose it at some point.
 *
 * It's not possible to use gpiod_line_bulk for querying and individual
 * gpiod_line instances for setting either, as a line can only be in use
 * once. So for the time being, just use a gpiod_line for each. */
struct GPIOPriv {
	int line_red, line_white;
};

static volatile uint32_t *gpio_base = (volatile uint32_t*) MAP_FAILED;
static int cable_gpio_close(CableHandle *h);

static int cable_gpio_open(CableHandle *h)
{
	if(gpio_base != MAP_FAILED) {
		ticables_warning(_("Can only have one instance of this cable."));
		return ERR_BUSY;
	}

	GPIOPriv *priv = (GPIOPriv*) calloc(1, sizeof(GPIOPriv));
	h->priv = priv;
	int mem_fd;

	if (!priv) {
		ticables_warning(_("Memory allocation failed."));
		goto err_cleanup;
	}

	priv->line_red = 23;
	priv->line_white = 24;

	mem_fd = open("/dev/mem", O_RDWR);
	if(mem_fd < 0) {
		goto err_cleanup;
	}

	gpio_base = (volatile uint32_t*)mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, 0x3F000000 + 0x200000);
	close(mem_fd);

	if (gpio_base == MAP_FAILED) {
		goto err_cleanup;
	}

	return ERR_NO_ERROR;

	err_cleanup:
	cable_gpio_close(h);
	return ERR_TTY_OPEN;
}

static int cable_gpio_close(CableHandle *h)
{
	GPIOPriv *priv = (GPIOPriv*) h->priv;
	if (!priv)
		return 0;

	if (gpio_base != MAP_FAILED) {
		munmap((void*) gpio_base, 4096);
		gpio_base = (volatile uint32_t*) MAP_FAILED;
	}

	free(h->priv);
	h->priv = NULL;
	return 0;
}

static int set_line(int line, bool b)
{
	if(b) // Set as input
		gpio_base[2] &= ~(0b111 << ((line - 20)*3));
	else // Set as output
		gpio_base[2] |= 1 << ((line - 20)*3);

	return 0;
}

static int cable_gpio_set_red_wire(CableHandle *h, int b)
{
	GPIOPriv *priv = (GPIOPriv*) h->priv;
	set_line(priv->line_red, b);
	return 0;
}

static int cable_gpio_set_white_wire(CableHandle *h, int b)
{
	GPIOPriv *priv = (GPIOPriv*) h->priv;
	set_line(priv->line_white, b);
	return 0;
}

static int cable_gpio_set_raw(CableHandle *h, int state)
{
	GPIOPriv *priv = (GPIOPriv*) h->priv;
	set_line(priv->line_white, state & BIT_WHITE);
	set_line(priv->line_red, state & BIT_RED);
	return 0;
}

/* Get the current level of the GPIO lines given in mask, store in *state.
 * The mask is used to avoid unnecessary ioctl calls. */
static int cable_gpio_get_raw_masked(CableHandle *h, int *state, int mask)
{
	*state = (gpio_base[13] >> 23) & mask;
	return 0;
}

static int cable_gpio_get_raw(CableHandle *h, int *state)
{
	return cable_gpio_get_raw_masked(h, state, 0b11);
}

static int cable_gpio_get_red_wire(CableHandle *h)
{
	int state = 0;
	cable_gpio_get_raw_masked(h, &state, BIT_RED);
	return !!(state & BIT_RED);
}

static int cable_gpio_get_white_wire(CableHandle *h)
{
	int state = 0;
	cable_gpio_get_raw_masked(h, &state, BIT_WHITE);
	return !!(state & BIT_WHITE);
}

// Wait until cable_gpio_get_raw() & mask == want.
static int wait_for_line_state(CableHandle *h, int mask, int want)
{
	tiTIME clk;
	bool to_started = false;
	int attempts = 0;

	while (true) {
		int state;
		int ret = cable_gpio_get_raw_masked(h, &state, mask);
		if (ret != 0)
			return ret;

		if ((state & mask) == want)
			return 0;

		// Syscalls are expensive, so avoid unnecessary calls to
		// TO_START/TO_ELAPSED.
		if(!to_started && attempts < 1000)
			attempts++;
		else if (!to_started) {
			TO_START(clk);
			to_started = true;
		} else if (TO_ELAPSED(clk, h->timeout))
			return ERR_READ_TIMEOUT; // TODO: What if called from put?
	}
}

static int cable_gpio_probe(CableHandle *h)
{
	int ret;

	// Stop pulling lines low
	ret = cable_gpio_set_raw(h, 0b11);
	if (ret != 0)
		return ret;

	// Wait for them to read high
	return wait_for_line_state(h, 0b11, 0b11);
}

static int cable_gpio_reset(CableHandle *h)
{
	return cable_gpio_probe(h);
}

static int cable_gpio_put(CableHandle *h, uint8_t *data, uint32_t len)
{
	for(unsigned int off = 0; off < len; off++) {
		const uint8_t byte = data[off];
		int ret;

		for (int bit = 0; bit < 8; bit++)  {
			if (byte & (1 << bit)) {
				cable_gpio_set_white_wire(h, 0);
				ret = wait_for_line_state(h, BIT_RED, 0);
				if (ret)
					return ret;
				cable_gpio_set_white_wire(h, 1);
				ret = wait_for_line_state(h, BIT_RED, BIT_RED);
				if (ret)
					return ret;
			} else {
				cable_gpio_set_red_wire(h, 0);
				ret = wait_for_line_state(h, BIT_WHITE, 0);
				if (ret)
					return ret;
				cable_gpio_set_red_wire(h, 1);
				ret = wait_for_line_state(h, BIT_WHITE, BIT_WHITE);
				if (ret)
					return ret;
			}
		}
	}

	return 0;
}

static int cable_gpio_get(CableHandle *h, uint8_t *data, uint32_t len)
{
	for(unsigned int off = 0; off < len; off++) {
		uint8_t byte = 0;

		for (int bit = 0; bit < 8; bit++)  {
			tiTIME clk;
			bool to_started = false;
			int attempts = 0;
			int state;
			int ret;

			// Wait until a line is low
			while (true) {
				ret = cable_gpio_get_raw(h, &state);
				if (ret != 0)
					return ret;

				if (state != 0b11)
					break;

				if(!to_started && attempts < 10000)
					attempts++;
				else if(!to_started) {
					TO_START(clk);
					to_started = true;
				} else if (TO_ELAPSED(clk, h->timeout))
					return ERR_READ_TIMEOUT;
			}

			byte >>= 1;

			if (state == BIT_RED) {
				// White line low -> 1 bit
				byte |= 0x80;
				cable_gpio_set_red_wire(h, 0);
				ret = wait_for_line_state(h, BIT_WHITE, BIT_WHITE);
				if (ret)
					return ret;
				cable_gpio_set_red_wire(h, 1);
			} else {
				cable_gpio_set_white_wire(h, 0);
				ret = wait_for_line_state(h, BIT_RED, BIT_RED);
				if (ret)
					return ret;
				cable_gpio_set_white_wire(h, 1);
			}

			/* It's impossible to tell apart whether the line is still
			 * held low by us or whether the next bit was sent. Wait
			 * a bit to be sure. */
			int dummy;
			for(int i = 0; i < h->delay; ++i)
				cable_gpio_get_raw(h, &dummy);
		}

		data[off] = byte;
	}

	return 0;
}

static int cable_gpio_check(CableHandle *h, int *status)
{
	int state = 0;
	cable_gpio_get_raw(h, &state);
	*status = state == 0b11 ? STATUS_NONE : (STATUS_RX | STATUS_TX);
	return 0;
}

extern const CableFncts cable_gpio =
{
	CABLE_GPIO,
	"GPIO",
	N_("RPi GPIO Link"),
	N_("Cable using RPi GPIO pins 16 and 18"),
	1,
	NULL,
	&cable_gpio_open, &cable_gpio_close, &cable_gpio_reset, &cable_gpio_probe, NULL,
	&cable_gpio_put, &cable_gpio_get, &cable_gpio_check,
	&cable_gpio_set_red_wire, &cable_gpio_set_white_wire,
	&cable_gpio_get_red_wire, &cable_gpio_get_white_wire,
	&cable_gpio_set_raw, &cable_gpio_get_raw,
	NULL,
	NULL
};
