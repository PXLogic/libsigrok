/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2010 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Olivier Fauchon <olivier@aixmarseille.com>
 * Copyright (C) 2012 Alexandru Gagniuc <mr.nuke.me@gmail.com>
 * Copyright (C) 2015 Bartosz Golaszewski <bgolaszewski@baylibre.com>
 * Copyright (C) 2019 Frank Stettner <frank-stettner@gmx.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"

#define ANALOG_SAMPLES_PER_PERIOD 20

static const uint8_t pattern_sigrok[] = {
	0x4c, 0x92, 0x92, 0x92, 0x64, 0x00, 0x00, 0x00,
	0x82, 0xfe, 0xfe, 0x82, 0x00, 0x00, 0x00, 0x00,
	0x7c, 0x82, 0x82, 0x92, 0x74, 0x00, 0x00, 0x00,
	0xfe, 0x12, 0x12, 0x32, 0xcc, 0x00, 0x00, 0x00,
	0x7c, 0x82, 0x82, 0x82, 0x7c, 0x00, 0x00, 0x00,
	0xfe, 0x10, 0x28, 0x44, 0x82, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xbe, 0xbe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const uint8_t pattern_squid[128][128 / 8] = {
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xe0, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xe1, 0x01, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xe1, 0x01, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xe3, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xe3, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xc3, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcf, 0xc7, 0x03, },
	{ 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0xc7, 0x03, },
	{ 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x87, 0x03, },
	{ 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0xc7, 0x03, },
	{ 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0xcf, 0x03, },
	{ 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xcf, 0x03, },
	{ 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0xfe, 0x01, },
	{ 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0xfe, 0x01, },
	{ 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xfc, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0xc0, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x80, 0x01, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0xfe, 0xff, 0x03, },
	{ 0x00, 0x00, 0x07, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xfe, 0xff, 0x03, },
	{ 0x00, 0x00, 0x1c, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xfe, 0xff, 0x03, },
	{ 0x00, 0x00, 0x78, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xfe, 0xff, 0x03, },
	{ 0x00, 0x00, 0xe0, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0xfe, 0xff, 0x03, },
	{ 0x00, 0x00, 0xc0, 0x03, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x07, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x1c, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0xf8, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, },
	{ 0x00, 0x00, 0x00, 0xf0, 0x01, 0x38, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x00, 0xf0, 0x1f, 0x1c, },
	{ 0x00, 0x00, 0x00, 0xe0, 0x07, 0x70, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x0f, 0x00, 0xfc, 0x3f, 0x3c, },
	{ 0x80, 0x03, 0x00, 0xc0, 0x0f, 0xe0, 0x00, 0x00, 0x80, 0xff, 0xff, 0x3f, 0x00, 0xfc, 0x7f, 0x7c, },
	{ 0x00, 0x1e, 0x00, 0x00, 0x1f, 0xc0, 0x01, 0x00, 0xc0, 0xff, 0xff, 0x7f, 0x00, 0xfe, 0xff, 0x7c, },
	{ 0x00, 0xf0, 0x01, 0x00, 0x7c, 0x80, 0x03, 0x00, 0xc0, 0xff, 0xff, 0xff, 0x00, 0xfe, 0xff, 0x7c, },
	{ 0x00, 0xc0, 0x0f, 0x00, 0xf0, 0x00, 0x07, 0x00, 0xc0, 0xff, 0xff, 0xff, 0x00, 0x3f, 0xf8, 0x78, },
	{ 0x00, 0x00, 0x3e, 0x00, 0xc0, 0x03, 0x0e, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xf0, 0xf0, },
	{ 0x00, 0x00, 0xf0, 0x07, 0x80, 0x07, 0x3c, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xf0, 0xf0, },
	{ 0x00, 0x00, 0x80, 0x3f, 0x00, 0x1e, 0x78, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xe0, 0xf0, },
	{ 0x00, 0x00, 0x00, 0xff, 0x00, 0x7c, 0xe0, 0x01, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xe0, 0xf0, },
	{ 0x00, 0x00, 0x00, 0xfc, 0x03, 0xf0, 0xc1, 0x07, 0xf0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xf0, 0xf0, },
	{ 0x00, 0x00, 0x00, 0xe0, 0x1f, 0xc0, 0x03, 0x1f, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xf0, 0x78, },
	{ 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x1f, 0xfc, 0xe0, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0x7f, },
	{ 0xf8, 0x03, 0x00, 0x00, 0xf0, 0x07, 0xfe, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0x7f, },
	{ 0x80, 0xff, 0x01, 0x00, 0x80, 0x3f, 0xf0, 0x8f, 0xff, 0xff, 0xff, 0xff, 0x00, 0xfe, 0xff, 0x3f, },
	{ 0x00, 0xf0, 0xff, 0x07, 0x00, 0xfc, 0x83, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xfe, 0xff, 0x3f, },
	{ 0x00, 0x00, 0xfc, 0x7f, 0x00, 0xe0, 0x7f, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x00, 0xfe, 0xff, 0x1f, },
	{ 0x00, 0x00, 0xc0, 0xff, 0x1f, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xfc, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0xf8, 0xff, 0x07, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x0f, 0xe0, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0x03, },
	{ 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0x03, },
	{ 0x00, 0x10, 0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x0f, 0xe0, 0xff, 0xff, 0xff, 0xff, 0x00, 0x7c, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0xf8, 0xff, 0x07, 0xc0, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x3c, 0x00, 0x00, },
	{ 0x00, 0x00, 0xc0, 0xff, 0x1f, 0x00, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1e, 0x00, 0x00, },
	{ 0x00, 0x00, 0xf8, 0x7f, 0x00, 0xe0, 0x7f, 0xfc, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1e, 0x00, 0x00, },
	{ 0x00, 0xf0, 0xff, 0x07, 0x00, 0xfc, 0x83, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x0f, 0x00, 0x00, },
	{ 0x80, 0xff, 0x01, 0x00, 0x80, 0x3f, 0xf0, 0x8f, 0xff, 0xff, 0xff, 0xff, 0x01, 0x0f, 0x00, 0x00, },
	{ 0xf8, 0x03, 0x00, 0x00, 0xf0, 0x07, 0xfe, 0xf0, 0xff, 0xff, 0xff, 0xff, 0x00, 0x0f, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x1f, 0xfc, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x0f, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0xe0, 0x1f, 0xc0, 0x07, 0x1f, 0xf0, 0xff, 0xff, 0xff, 0x00, 0x06, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0xfc, 0x03, 0xf0, 0xc1, 0x07, 0xf0, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0xff, 0x00, 0x7c, 0xe0, 0x01, 0xf0, 0xff, 0xff, 0xff, 0x01, 0xe0, 0x1f, 0x00, },
	{ 0x00, 0x00, 0x80, 0x3f, 0x00, 0x1e, 0x78, 0x00, 0xf0, 0xff, 0xff, 0xff, 0x01, 0xf0, 0x7f, 0x00, },
	{ 0x00, 0x00, 0xf0, 0x0f, 0x80, 0x07, 0x3c, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x01, 0xfc, 0xff, 0x00, },
	{ 0x00, 0x00, 0x3e, 0x00, 0xc0, 0x03, 0x0e, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x01, 0xfc, 0xff, 0x01, },
	{ 0x00, 0xc0, 0x0f, 0x00, 0xf0, 0x00, 0x07, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x01, 0xfe, 0xff, 0x01, },
	{ 0x00, 0xf0, 0x01, 0x00, 0x7c, 0x80, 0x03, 0x00, 0xe0, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0x03, },
	{ 0x00, 0x3e, 0x00, 0x00, 0x1f, 0xc0, 0x01, 0x00, 0x80, 0xff, 0xff, 0xff, 0x00, 0x1f, 0xe0, 0x03, },
	{ 0x80, 0x03, 0x00, 0xc0, 0x0f, 0xe0, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x0f, 0xc0, 0x03, },
	{ 0x00, 0x00, 0x00, 0xe0, 0x07, 0x70, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x00, 0x0f, 0xc0, 0x03, },
	{ 0x00, 0x00, 0x00, 0xf0, 0x01, 0x38, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0x00, 0x0f, 0x80, 0x03, },
	{ 0x00, 0x00, 0x00, 0xf8, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x03, },
	{ 0x00, 0x00, 0x00, 0x1c, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 0x03, },
	{ 0x00, 0x00, 0x00, 0x07, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xe0, 0x03, },
	{ 0x00, 0x00, 0xc0, 0x03, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x03, },
	{ 0x00, 0x00, 0xe0, 0x00, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x01, },
	{ 0x00, 0x00, 0x78, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x01, },
	{ 0x00, 0x00, 0x1c, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0x00, },
	{ 0x00, 0x00, 0x06, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x7f, 0x00, },
	{ 0x00, 0x80, 0x03, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1f, 0x00, },
	{ 0x00, 0xc0, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x01, },
	{ 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x03, },
	{ 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0x00, },
	{ 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0x00, },
	{ 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x0f, 0x00, },
	{ 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x1f, 0x00, },
	{ 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x01, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfc, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf8, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf0, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, },
};

SR_PRIV void demo_generate_analog_pattern(struct dev_context *devc)
{
	double t, frequency;
	float amplitude, offset;
	struct analog_pattern *pattern;
	unsigned int num_samples, i;
	float value;
	int last_end;

	num_samples = ANALOG_BUFSIZE / sizeof(float);
	frequency = (double) devc->cur_samplerate / ANALOG_SAMPLES_PER_PERIOD;
	amplitude = DEFAULT_ANALOG_AMPLITUDE;
	offset = DEFAULT_ANALOG_OFFSET;

	/*
	 * FIXME: We actually need only one period. A ringbuffer would be
	 * useful here.
	 * Make sure the number of samples we put out is an integer
	 * multiple of our period size.
	 */

	/* PATTERN_SQUARE: */
	sr_dbg("Generating %s pattern.", analog_pattern_str[PATTERN_SQUARE]);
	pattern = g_malloc(sizeof(struct analog_pattern));
	value = amplitude;
	last_end = 0;
	for (i = 0; i < num_samples; i++) {
		if (i % 5 == 0)
			value = -value;
		if (i % 10 == 0)
			last_end = i;
		pattern->data[i] = value + offset;
	}
	pattern->num_samples = last_end;
	devc->analog_patterns[PATTERN_SQUARE] = pattern;

	/* Readjusting num_samples for all other patterns. */
	while (num_samples % ANALOG_SAMPLES_PER_PERIOD != 0)
		num_samples--;

	/* PATTERN_SINE: */
	sr_dbg("Generating %s pattern.", analog_pattern_str[PATTERN_SINE]);
	pattern = g_malloc(sizeof(struct analog_pattern));
	for (i = 0; i < num_samples; i++) {
		t = (double) i / (double) devc->cur_samplerate;
		pattern->data[i] = sin(2 * G_PI * frequency * t) * amplitude + offset;
	}
	pattern->num_samples = last_end;
	devc->analog_patterns[PATTERN_SINE] = pattern;

	/* PATTERN_TRIANGLE: */
	sr_dbg("Generating %s pattern.", analog_pattern_str[PATTERN_TRIANGLE]);
	pattern = g_malloc(sizeof(struct analog_pattern));
	for (i = 0; i < num_samples; i++) {
		t = (double) i / (double) devc->cur_samplerate;
		pattern->data[i] = (2 / G_PI) * asin(sin(2 * G_PI * frequency * t)) *
			amplitude + offset;
	}
	pattern->num_samples = last_end;
	devc->analog_patterns[PATTERN_TRIANGLE] = pattern;

	/* PATTERN_SAWTOOTH: */
	sr_dbg("Generating %s pattern.", analog_pattern_str[PATTERN_SAWTOOTH]);
	pattern = g_malloc(sizeof(struct analog_pattern));
	for (i = 0; i < num_samples; i++) {
		t = (double) i / (double) devc->cur_samplerate;
		pattern->data[i] = 2 * ((t * frequency) - floor(0.5f + t * frequency)) *
			amplitude + offset;
	}
	pattern->num_samples = last_end;
	devc->analog_patterns[PATTERN_SAWTOOTH] = pattern;

	/* PATTERN_ANALOG_RANDOM */
	/* Data not filled here, will be generated in send_analog_packet(). */
	pattern = g_malloc(sizeof(struct analog_pattern));
	pattern->num_samples = last_end;
	devc->analog_patterns[PATTERN_ANALOG_RANDOM] = pattern;
}

SR_PRIV void demo_free_analog_pattern(struct dev_context *devc)
{
	g_free(devc->analog_patterns[PATTERN_SQUARE]);
	g_free(devc->analog_patterns[PATTERN_SINE]);
	g_free(devc->analog_patterns[PATTERN_TRIANGLE]);
	g_free(devc->analog_patterns[PATTERN_SAWTOOTH]);
	g_free(devc->analog_patterns[PATTERN_ANALOG_RANDOM]);
}

static uint64_t encode_number_to_gray(uint64_t nr)
{
	return nr ^ (nr >> 1);
}

/* Fast xorshift32 PRNG — ~50x faster than rand() on Windows.
 * Generates a full 32-bit random word per call; we extract bytes from it
 * to fill buffers 4 bytes at a time, amortizing the call cost. */
static inline uint32_t demo_xorshift32(uint32_t *state)
{
	uint32_t x = *state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*state = x;
	return x;
}

/* Fill a buffer with fast random bytes using xorshift32.
 * Generates 4 random bytes per PRNG iteration and stores them at once,
 * with a scalar tail for the remaining 1-3 bytes. */
static void demo_fill_random(uint32_t *prng_state, uint8_t *buf, size_t len)
{
	size_t i = 0;
	while (i + 4 <= len) {
		uint32_t r = demo_xorshift32(prng_state);
		buf[i]     = (uint8_t)(r);
		buf[i + 1] = (uint8_t)(r >> 8);
		buf[i + 2] = (uint8_t)(r >> 16);
		buf[i + 3] = (uint8_t)(r >> 24);
		i += 4;
	}
	if (i < len) {
		uint32_t r = demo_xorshift32(prng_state);
		while (i < len) {
			buf[i++] = (uint8_t)(r);
			r >>= 8;
		}
	}
}

static void set_logic_data(uint64_t bits, uint8_t *data, size_t len)
{
	while (len--) {
		*data++ = bits & 0xff;
		bits >>= 8;
	}
}

/* I2C pattern generator: produces a stream of valid I2C bus traffic
 * on ch0(SCL) and ch1(SDA).
 *
 * The generator uses a frame-based approach. Each I2C transaction is:
 *   2 idle bits + START(1 bit) + ADDR(8 bits) + ACK(1 bit) + DATA(8 bits)
 *   + ACK(1 bit) + ... + DATA(8 bits) + ACK(1 bit) + STOP(1 bit) + 2 idle
 *
 * Frame layout (bit indices, after 2 leading idle bits):
 *   bit 0-1:    bus idle (SCL=1, SDA=1)
 *   bit 2:      START  (SDA falling while SCL high)
 *   bits 3-10:  ADDRESS (7-bit + R/W, MSB first)
 *   bit 11:     ACK (slave pulls SDA low)
 *   bits 12-19: DATA byte 0 (MSB first)
 *   bit 20:     ACK
 *   bits 21-28: DATA byte 1
 *   bit 29:     ACK
 *   bits 30-37: DATA byte 2
 *   bit 38:     ACK
 *   bit 39:     STOP  (SDA rising while SCL high)
 *   bits 40-41: bus idle (SCL=1, SDA=1)
 * Total: 42 bits per transaction (3 data bytes)
 *
 * Address is fixed at 0x50 (standard 24C02 EEPROM device address).
 * Data bytes form an EEPROM write sequence:
 *   DATA0 = word address (increments each transaction)
 *   DATA1 = data value 1 (0x10 + low nibble of frame)
 *   DATA2 = data value 2 (0x20 + low nibble of frame)
 *
 * Bit encoding within each I2C bit time (I2C_SPB samples):
 *   For data bits: first half SCL=low (SDA can change), second half
 *   SCL=high (SDA stable). SDA switches only during the SCL-low window,
 *   and only after the first SPB/8 samples of that window (hold time), so
 *   SDA never transitions while SCL is high — this is what the i2c_c
 *   decoder relies on to detect START/STOP (spec fix-demo-pattern-bus-timing
 *   requirement R2).
 *   For START: SCL stays high, SDA goes high→low.
 *   For STOP: SCL stays high, SDA goes low→high.
 */
#define I2C_FRAME_BITS		42
#define I2C_DATA_BYTES		3

static void i2c_get_bit(uint32_t bit_idx, uint32_t bit_phase,
	uint32_t bittime, uint8_t *scl, uint8_t *sda)
{
	uint32_t frame_bit = bit_idx % I2C_FRAME_BITS;
	uint32_t frame_num = bit_idx / I2C_FRAME_BITS;
	uint8_t addr = 0x50; /* Fixed EEPROM device address (24C02) */
	uint8_t is_high_phase = (bit_phase >= bittime / 2) ? 1 : 0;
	uint8_t in_hold = (bit_phase < bittime / 8) ? 1 : 0;
	uint8_t scl_val = 0, sda_val = 0;

	/* Default: SCL toggles high in second half, low in first half.
	 * SDA holds its value during SCL high, changes during SCL low
	 * (only after the SPB/8 hold window, so never while SCL is high). */

	if (frame_bit < 2 || frame_bit >= 40) {
		/* Leading/trailing bus idle: SCL=1, SDA=1 */
		scl_val = 1; sda_val = 1;
	} else if (frame_bit == 2) {
		/* START: SDA falls while SCL high.
		 * First quarter: SDA high, SCL high (idle)
		 * Second quarter: SDA low, SCL high (START edge)
		 * Rest: SCL goes low to prepare for first data bit */
		if (bit_phase < bittime / 4) {
			scl_val = 1; sda_val = 1;
		} else if (bit_phase < bittime / 2) {
			scl_val = 1; sda_val = 0;
		} else {
			scl_val = 0; sda_val = 0;
		}
	} else if (frame_bit == 39) {
		/* STOP: SDA rises while SCL high.
		 * First quarter: SCL low, SDA low
		 * Second quarter: SCL high, SDA low
		 * Rest: SCL high, SDA high (idle) */
		if (bit_phase < bittime / 4) {
			scl_val = 0; sda_val = 0;
		} else if (bit_phase < bittime / 2) {
			scl_val = 1; sda_val = 0;
		} else {
			scl_val = 1; sda_val = 1;
		}
	} else if (frame_bit == 11 || frame_bit == 20 ||
		   frame_bit == 29 || frame_bit == 38) {
		/* ACK bit: slave pulls SDA low. SCL toggles normally. */
		scl_val = is_high_phase;
		sda_val = (in_hold && is_high_phase) ? 1 : 0;
	} else {
		/* Data bit: determine which bit of address or data.
		 * Bits 3-10: address (7-bit MSB first + R/W at bit 10).
		 * R/W is always 0 (write) for simplicity.
		 * Bits 12-19: data byte 0
		 * Bits 21-28: data byte 1
		 * Bits 30-37: data byte 2 */
		uint8_t byte_val = 0;
		uint8_t prev_val = 1; /* idle level before a data bit */
		int bit_in_byte = 0;

		if (frame_bit >= 3 && frame_bit <= 10) {
			/* Address: bits 3-9 are addr[6:0], bit 10 is R/W=0 */
			if (frame_bit <= 9) {
				bit_in_byte = 9 - frame_bit; /* MSB first */
				byte_val = (addr >> bit_in_byte) & 1;
				if (bit_in_byte < 6)
					prev_val = (addr >> (bit_in_byte + 1)) & 1;
			} else {
				byte_val = 0; /* R/W = write */
			}
		} else if (frame_bit >= 12 && frame_bit <= 19) {
			/* EEPROM word address (increments each transaction) */
			uint8_t data0 = (uint8_t)(frame_num & 0xFF);
			bit_in_byte = 19 - frame_bit;
			byte_val = (data0 >> bit_in_byte) & 1;
			if (bit_in_byte < 7)
				prev_val = (data0 >> (bit_in_byte + 1)) & 1;
		} else if (frame_bit >= 21 && frame_bit <= 28) {
			/* EEPROM data byte 1 */
			uint8_t data1 = (uint8_t)(0x10 + (frame_num & 0x0F));
			bit_in_byte = 28 - frame_bit;
			byte_val = (data1 >> bit_in_byte) & 1;
			if (bit_in_byte < 7)
				prev_val = (data1 >> (bit_in_byte + 1)) & 1;
		} else if (frame_bit >= 30 && frame_bit <= 37) {
			/* EEPROM data byte 2 */
			uint8_t data2 = (uint8_t)(0x20 + (frame_num & 0x0F));
			bit_in_byte = 37 - frame_bit;
			byte_val = (data2 >> bit_in_byte) & 1;
			if (bit_in_byte < 7)
				prev_val = (data2 >> (bit_in_byte + 1)) & 1;
		}

		/* SCL toggles: low in first half, high in second half.
		 * SDA is stable during SCL high; it switches to the current
		 * bit value only after the SPB/8 hold window (SCL low), keeping
		 * the previous bit's value before that — so SDA never changes
		 * while SCL is high. */
		scl_val = is_high_phase;
		if (is_high_phase)
			sda_val = byte_val;
		else
			sda_val = in_hold ? prev_val : byte_val;
	}

	*scl = scl_val;
	*sda = sda_val;
}

/* ---- Mixed pattern generators: SPI, UART, CAN, PWM, I2S, MIPI DSI, SWD ----
 *
 * PATTERN_MIXED fills 16 channels with 8 protocol types:
 *   ch0-1:   I2C    (SCL, SDA)
 *   ch2-5:   SPI    (CS, SCLK, MOSI, MISO)
 *   ch6:     UART   (RX)
 *   ch7:     CAN    (RX)
 *   ch8:     PWM    (DATA)
 *   ch9-11:  I2S    (SCK, WS, SD)
 *   ch12-13: MIPI DSI (D0N, D0P)
 *   ch14-15: SWD    (SWDIO, SWCLK)
 *
 * 8 buses, each with C + Python decoder = 16 decoders total.
 */

/* SPI Flash READ command frame (idle-first layout):
 *   12 idle bits (CS=1, SCLK=0, lines idle) + 40 transfer bits (CS=0).
 * Transfer bytes (MSB first):
 *   byte 0: 0x03 (READ Data command)
 *   byte 1: 0x00 (address bits 23-16)
 *   byte 2: 0x00 (address bits 15-8)
 *   byte 3: frame_num & 0xFF (address bits 7-0, increments each frame)
 *   byte 4: 0x10 + (frame_num & 0x0F) (data byte from flash, on MISO)
 * CS stays LOW for all 5 bytes (40 bits), then 12 idle bits (CS high).
 * Total: 52 bits per frame.
 *
 * Data lines (MOSI/MISO) switch only during the SCLK-low window and after
 * the first SPB/8 samples of it (hold time), so they are stable on the
 * SCLK rising edge — matching SPI mode 0 sampling (spec requirement R2).
 *
 * This produces valid SPI Flash traffic that the spiflash decoder
 * can decode: it sees READ commands with incrementing addresses
 * and recognizable data bytes on MISO. */
#define SPI_FLASH_CMD_BYTES	5
#define SPI_FLASH_IDLE_BITS	12
#define SPI_FRAME_BITS		(SPI_FLASH_IDLE_BITS + SPI_FLASH_CMD_BYTES * 8)

static void spi_get_bits(uint32_t bit_idx, uint32_t bit_phase,
	uint32_t bittime, uint32_t data_offset,
	uint8_t *cs, uint8_t *sclk, uint8_t *mosi, uint8_t *miso)
{
	uint32_t frame_bit = bit_idx % SPI_FRAME_BITS;
	uint32_t frame_num = bit_idx / SPI_FRAME_BITS;
	uint8_t is_high = (bit_phase >= bittime / 2) ? 1 : 0;
	uint8_t in_hold = (bit_phase < bittime / 8) ? 1 : 0;
	uint8_t addr_low = (uint8_t)((frame_num + data_offset) & 0xFF);

	if (frame_bit < SPI_FLASH_IDLE_BITS) {
		/* Idle gap: CS high, SCLK low, lines idle */
		*cs = 1; *sclk = 0; *mosi = 0; *miso = 0;
		return;
	}

	/* CS low during command + address + data bytes */
	*cs = 0;
	*sclk = is_high;

	uint32_t byte_idx = (frame_bit - SPI_FLASH_IDLE_BITS) / 8;
	int bit_in_byte = 7 - ((frame_bit - SPI_FLASH_IDLE_BITS) % 8); /* MSB first */
	uint8_t mosi_byte = 0xFF, miso_byte = 0xFF;

	switch (byte_idx) {
	case 0: /* Command byte: 0x03 = READ */
		mosi_byte = 0x03;
		break;
	case 1: /* Address high byte */
		mosi_byte = 0x00;
		break;
	case 2: /* Address mid byte */
		mosi_byte = 0x00;
		break;
	case 3: /* Address low byte (increments) */
		mosi_byte = addr_low;
		break;
	case 4: /* Data byte from flash (on MISO) */
		mosi_byte = 0xFF; /* dummy on MOSI */
		miso_byte = (uint8_t)(0x10 + ((frame_num + data_offset) & 0x0F));
		break;
	default:
		break;
	}

	uint8_t mosi_bit = (mosi_byte >> bit_in_byte) & 1;
	uint8_t miso_bit = (miso_byte >> bit_in_byte) & 1;
	/* Switch data lines during the SCLK-low window only (after SPB/8),
	 * keep them stable while SCLK is high — no transition on the rising
	 * edge that mode-0 sampling uses. */
	if (is_high) {
		*mosi = mosi_bit;
		*miso = miso_bit;
	} else {
		/* During SCLK low: hold previous value for the first SPB/8
		 * samples, then adopt the new bit (edge happens mid-low). */
		uint8_t prev_mosi = (bit_in_byte < 7)
			? ((mosi_byte >> (bit_in_byte + 1)) & 1) : 0xFF;
		uint8_t prev_miso = (bit_in_byte < 7)
			? ((miso_byte >> (bit_in_byte + 1)) & 1) : 0xFF;
		*mosi = in_hold ? prev_mosi : mosi_bit;
		*miso = in_hold ? prev_miso : miso_bit;
	}
}

/* UART frame: 12 bit times = 2 leading mark + 1 start + 8 data LSB-first
 * + 1 stop + 2 trailing mark. The leading mark (idle) gives the stream a
 * high level before the first start edge so the decoder can resync on the
 * first falling edge (spec requirement R3); the trailing mark gives ≥2 bit
 * times of idle between frames. */
#define UART_FRAME_BITS 12

static uint8_t uart_get_rx_bit(uint32_t bit_idx, uint32_t bit_phase,
	uint32_t bittime, uint32_t rx_offset)
{
	uint32_t uart_bit = bit_idx % UART_FRAME_BITS;
	uint32_t uart_frame = bit_idx / UART_FRAME_BITS;
	(void)bit_phase;
	(void)bittime;

	if (uart_bit < 2)
		return 1; /* Leading mark (stream idle before start bit) */
	if (uart_bit == 2)
		return 0; /* Start bit */
	if (uart_bit == 11)
		return 1; /* Stop bit */
	/* Trailing mark idle: bits 10 (already past stop at 11) handled by
	 * the stop check; remaining bits after stop are mark (1). */
	if (uart_bit > 11)
		return 1;
	int bit_in_byte = (int)uart_bit - 3;
	uint8_t rx_data = (uint8_t)((uart_frame + rx_offset) & 0xFF);
	rx_data ^= 0xAA;
	return (rx_data >> bit_in_byte) & 1;
}

/* CAN frame: simplified CAN 2.0 standard frame, 63 bits.
 * bit 0:      SOF (dominant/0)
 * bits 1-11:  Identifier (11 bits, MSB first)
 * bit 12:     RTR (0=data)
 * bit 13:     IDE (0=standard)
 * bit 14:     r0 (0)
 * bits 15-18: DLC (4 bits, value=2)
 * bits 19-34: Data (2 bytes, MSB first per byte)
 * bits 35-49: CRC (15 bits, alternating)
 * bit 50:     CRC delimiter (1)
 * bit 51:     ACK slot (0=acknowledged)
 * bit 52:     ACK delimiter (1)
 * bits 53-59: EOF (7 bits, all 1)
 * bits 60-62: IFS (3 bits, all 1)
 * No bit stuffing (data uses 0xAA/0x55 to avoid long runs). */
#define CAN_FRAME_BITS 63

static uint8_t can_get_bit(uint32_t bit_idx, uint32_t bit_phase,
	uint32_t bittime, uint32_t can_id_offset)
{
	uint32_t frame_bit = bit_idx % CAN_FRAME_BITS;
	uint32_t frame_num = bit_idx / CAN_FRAME_BITS;
	uint16_t can_id = (uint16_t)(0x100 + ((frame_num + can_id_offset) & 0x3FF));
	(void)bit_phase;
	(void)bittime;

	if (frame_bit == 0)
		return 0; /* SOF */
	if (frame_bit >= 1 && frame_bit <= 11) {
		int bit_pos = 10 - (frame_bit - 1);
		return (can_id >> bit_pos) & 1;
	}
	if (frame_bit == 12) return 0; /* RTR */
	if (frame_bit == 13) return 0; /* IDE */
	if (frame_bit == 14) return 0; /* r0 */
	if (frame_bit >= 15 && frame_bit <= 18) {
		int bit_pos = 3 - (frame_bit - 15);
		return (2 >> bit_pos) & 1; /* DLC=2 */
	}
	if (frame_bit >= 19 && frame_bit <= 34) {
		int byte_idx = (frame_bit - 19) / 8;
		int bit_in_byte = 7 - ((frame_bit - 19) % 8);
		uint8_t data = (byte_idx == 0)
			? (uint8_t)(0xAA ^ (frame_num & 0xFF))
			: (uint8_t)(0x55 ^ (frame_num & 0xFF));
		return (data >> bit_in_byte) & 1;
	}
	if (frame_bit >= 35 && frame_bit <= 49)
		return ((frame_bit - 35) & 1) ? 1 : 0; /* CRC */
	if (frame_bit == 50) return 1; /* CRC delimiter */
	if (frame_bit == 51) return 0; /* ACK slot */
	if (frame_bit == 52) return 1; /* ACK delimiter */
	return 1; /* EOF + IFS (bits 53-62) */
}

/* JTAG cycle: 19 bit times.
 * TMS sequence: TLR(5) → Idle(1) → SelDR(1) → CapDR(1) → ShiftDR(8) → Exit1(1) → UpdDR(1) → Idle(1)
 * TCK: toggles every half bit time.
 * TDI: shifts data during Shift-DR phase.
 * TDO: per-device output during Shift-DR phase. */
#define JTAG_CYCLE_BITS 19

G_GNUC_UNUSED static void jtag_get_bits(uint32_t bit_idx, uint32_t bit_phase,
	uint32_t bittime, int tdo_device_idx,
	uint8_t *tck, uint8_t *tms, uint8_t *tdi, uint8_t *tdo)
{
	static const uint8_t tms_seq[JTAG_CYCLE_BITS] = {
		1,1,1,1,1, /* Test-Logic-Reset */
		0,         /* Run-Test/Idle */
		1,         /* Select-DR-Scan */
		0,         /* Capture-DR */
		0,0,0,0,0,0,0,0, /* Shift-DR (8 clocks) */
		1,         /* Exit1-DR */
		0,         /* Update-DR */
		0          /* Run-Test/Idle */
	};
	uint32_t jtag_bit = bit_idx % JTAG_CYCLE_BITS;
	uint32_t jtag_cycle = bit_idx / JTAG_CYCLE_BITS;
	uint8_t is_high = (bit_phase >= bittime / 2) ? 1 : 0;

	*tck = is_high;
	*tms = tms_seq[jtag_bit];

	/* TDI: shift during Shift-DR (bits 8-15) */
	if (jtag_bit >= 8 && jtag_bit <= 15) {
		uint8_t tdi_data = (uint8_t)((jtag_cycle + 1) & 0xFF);
		int bit_pos = (jtag_bit - 8) % 8;
		*tdi = (tdi_data >> bit_pos) & 1;
	} else {
		*tdi = 0;
	}

	/* TDO: per-device data during Shift-DR */
	if (jtag_bit >= 8 && jtag_bit <= 15) {
		uint8_t tdo_data = (uint8_t)((jtag_cycle + tdo_device_idx * 16 + 0x80) & 0xFF);
		int bit_pos = (jtag_bit - 8) % 8;
		*tdo = (tdo_data >> bit_pos) & 1;
	} else {
		*tdo = 0;
	}
}

/* SWD transaction: 60 bit times.
 * bits 0-7:   idle (SWDIO=0)
 * bit 8:      start (1)
 * bit 9:      APnDP (0=AP)
 * bit 10:     RnW (1=read)
 * bits 11-12: address
 * bit 13:     turnaround
 * bits 14-16: ACK (001=OK)
 * bits 17-48: data (32 bits, LSB first)
 * bit 49:     parity
 * bit 50:     turnaround
 * bits 51-59: trailing idle */
#define SWD_TRANS_BITS 60

static void swd_get_bits(uint32_t bit_idx, uint32_t bit_phase,
	uint32_t bittime, uint32_t swd_offset,
	uint8_t *swdio, uint8_t *swclk)
{
	uint32_t swd_bit = bit_idx % SWD_TRANS_BITS;
	uint32_t swd_trans = bit_idx / SWD_TRANS_BITS;
	uint8_t is_high = (bit_phase >= bittime / 2) ? 1 : 0;

	*swclk = is_high;

	if (swd_bit < 8) {
		*swdio = 0; /* idle */
	} else if (swd_bit == 8) {
		*swdio = 1; /* start */
	} else if (swd_bit == 9) {
		*swdio = 0; /* APnDP */
	} else if (swd_bit == 10) {
		*swdio = 1; /* RnW = read */
	} else if (swd_bit >= 11 && swd_bit <= 12) {
		*swdio = ((swd_trans + swd_offset) >> (swd_bit - 11)) & 1;
	} else if (swd_bit == 13) {
		*swdio = 0; /* turnaround */
	} else if (swd_bit >= 14 && swd_bit <= 16) {
		*swdio = (swd_bit == 14) ? 1 : 0; /* ACK=001 */
	} else if (swd_bit >= 17 && swd_bit <= 48) {
		uint32_t data = 0xDEADBEEF + swd_trans + swd_offset;
		*swdio = (data >> (swd_bit - 17)) & 1;
	} else if (swd_bit == 49) {
		*swdio = 0; /* parity */
	} else if (swd_bit == 50) {
		*swdio = 0; /* turnaround */
	} else {
		*swdio = 0; /* trailing idle */
	}
}

/* PWM for mixed mode: 50% duty, period=100 samples (10 kHz at 1 MHz). */
static uint8_t pwm_mixed_get_bit(uint32_t sample_idx)
{
	return (sample_idx % 100) < 50 ? 1 : 0;
}

/* I2S frame: 64 bit times (32 left + 32 right channel).
 * SCK: toggles every half bit time (rising edge samples data).
 * WS: 0 for left channel, 1 for right, changes one SCK before first bit.
 * SD: MSB first, 16-bit audio in 32-bit slot (upper 16 bits). */
#define I2S_FRAME_BITS 64

static void i2s_get_bits(uint32_t bit_idx, uint32_t bit_phase,
	uint32_t bittime, uint32_t data_offset,
	uint8_t *sck, uint8_t *ws, uint8_t *sd)
{
	uint32_t frame_bit = bit_idx % I2S_FRAME_BITS;
	uint32_t frame_num = bit_idx / I2S_FRAME_BITS;
	uint8_t is_high = (bit_phase >= bittime / 2) ? 1 : 0;

	*sck = is_high;
	*ws = (frame_bit >= 32) ? 1 : 0;

	/* 16-bit audio sample MSB first in upper half of 32-bit slot */
	uint32_t bit_in_slot = frame_bit % 32;
	if (bit_in_slot < 16) {
		uint16_t sample;
		if (frame_bit < 32)
			sample = (uint16_t)((frame_num * 137 + data_offset) & 0xFFFF);
		else
			sample = (uint16_t)((frame_num * 137 + data_offset + 0x8000) & 0xFFFF);
		int bit_pos = 15 - bit_in_slot;
		*sd = (sample >> bit_pos) & 1;
	} else {
		*sd = 0;
	}
}

/* MIPI DSI LP data frame: 44 bit times.
 * LP states (D0N, D0P):
 *   LP-11 (1,1): idle / stop
 *   LP-01 (0,1): start, escape mode, data bit=1 edge
 *   LP-10 (1,0): data bit=0 edge
 *   LP-00 (0,0): turnaround, data valid
 *
 * Frame layout:
 *   bits 0-3:  LP-11 (idle)
 *   bit  4:    LP-01 (start — D0N falls while D0P high)
 *   bit  5:    LP-00 (turnaround)
 *   bit  6:    LP-01 (escape mode select)
 *   bit  7:    LP-00 (turnaround)
 *   bits 8-39: 2 bytes data, LSB first, each bit = 2 states (edge + valid)
 *   bits 40-43: LP-11 (stop) */
#define MIPI_DSI_FRAME_BITS 44
#define MIPI_DSI_DATA_BYTES 2

static void mipi_dsi_get_bits(uint32_t bit_idx, uint32_t bit_phase,
	uint32_t bittime, uint32_t data_offset,
	uint8_t *d0n, uint8_t *d0p)
{
	uint32_t frame_bit = bit_idx % MIPI_DSI_FRAME_BITS;
	uint32_t frame_num = bit_idx / MIPI_DSI_FRAME_BITS;
	(void)bit_phase;
	(void)bittime;

	if (frame_bit < 4) {
		/* Idle: LP-11 */
		*d0n = 1; *d0p = 1;
	} else if (frame_bit == 4) {
		/* Start: LP-01 (D0N falls while D0P high) */
		*d0n = 0; *d0p = 1;
	} else if (frame_bit == 5) {
		/* Turnaround: LP-00 */
		*d0n = 0; *d0p = 0;
	} else if (frame_bit == 6) {
		/* Escape mode: LP-01 */
		*d0n = 0; *d0p = 1;
	} else if (frame_bit == 7) {
		/* Turnaround: LP-00 */
		*d0n = 0; *d0p = 0;
	} else if (frame_bit >= 8 && frame_bit < 8 + MIPI_DSI_DATA_BYTES * 8 * 2) {
		/* Data: 2 bytes, LSB first, each bit = 2 states (edge + valid) */
		uint32_t data_idx = frame_bit - 8;
		uint32_t bit_pair = data_idx / 2;  /* which data bit (0-15) */
		uint32_t is_valid = data_idx % 2;  /* 0=edge, 1=valid */

		uint32_t byte_idx = bit_pair / 8;  /* 0 or 1 */
		uint32_t bit_in_byte = bit_pair % 8;  /* 0-7, LSB first */

		uint8_t data_byte = (byte_idx == 0)
			? (uint8_t)((frame_num * 31 + data_offset) & 0xFF)
			: (uint8_t)((frame_num * 31 + data_offset + 0x42) & 0xFF);
		uint8_t bit_val = (data_byte >> bit_in_byte) & 1;

		if (is_valid) {
			/* Data valid: LP-00 */
			*d0n = 0; *d0p = 0;
		} else {
			/* Data edge: bit=1 → LP-01, bit=0 → LP-10 */
			if (bit_val) {
				*d0n = 0; *d0p = 1;
			} else {
				*d0n = 1; *d0p = 0;
			}
		}
	} else {
		/* Stop: LP-11 */
		*d0n = 1; *d0p = 1;
	}
}

/* Generate a 32-bit sample for PATTERN_MIXED.
 * Returns one bit per channel (bit 0 = ch0, bit 31 = ch31).
 * ch0-15 carry 8 protocol types; ch16-31 mirror ch0-15.
 *
 * Channel layout (16 channels, 8 protocol types):
 *   ch0-1:   I2C     (SCL, SDA)
 *   ch2-5:   SPI     (CS, SCLK, MOSI, MISO)
 *   ch6:     UART    (RX)
 *   ch7:     CAN     (RX)
 *   ch8:     PWM     (DATA)
 *   ch9-11:  I2S     (SCK, WS, SD)
 *   ch12-13: MIPI DSI (D0N, D0P)
 *   ch14-15: SWD     (SWDIO, SWCLK)
 *   ch16-31: mirror of ch0-15
 */
static uint32_t mixed_get_sample(uint32_t bit_idx, uint32_t bit_phase,
	uint32_t bittime, uint32_t sample_idx)
{
	uint32_t val = 0;
	(void)bit_idx;
	(void)bit_phase;
	(void)bittime;

	/* Each protocol runs on its own samples-per-bit timebase (R1) so its
	 * bit boundaries fall at independent positions, plus a small per-bus
	 * phase offset so edges never align across buses. */
	uint32_t step = sample_idx;

	/* I2C: ch0=SCL, ch1=SDA */
	{
		uint8_t scl, sda;
		uint32_t p = (step + 0) % I2C_SPB;
		i2c_get_bit((step + 0) / I2C_SPB, p, I2C_SPB, &scl, &sda);
		val |= (uint32_t)scl << 0;
		val |= (uint32_t)sda << 1;
	}
	/* SPI: ch2=CS, ch3=SCLK, ch4=MOSI, ch5=MISO */
	{
		uint8_t cs, sclk, mosi, miso;
		uint32_t p = (step + 7) % SPI_SPB;
		spi_get_bits((step + 7) / SPI_SPB, p, SPI_SPB, 0, &cs, &sclk, &mosi, &miso);
		val |= (uint32_t)cs << 2;
		val |= (uint32_t)sclk << 3;
		val |= (uint32_t)mosi << 4;
		val |= (uint32_t)miso << 5;
	}
	/* UART: ch6=RX */
	{
		uint32_t p = (step + 13) % UART_SPB;
		val |= (uint32_t)uart_get_rx_bit((step + 13) / UART_SPB, p,
			UART_SPB, 0) << 6;
	}
	/* CAN: ch7=RX */
	{
		uint32_t p = (step + 19) % CAN_SPB;
		val |= (uint32_t)can_get_bit((step + 19) / CAN_SPB, p,
			CAN_SPB, 0) << 7;
	}
	/* PWM: ch8=DATA */
	val |= (uint32_t)pwm_mixed_get_bit(sample_idx) << 8;
	/* I2S: ch9=SCK, ch10=WS, ch11=SD */
	{
		uint8_t sck, ws, sd;
		uint32_t p = (step + 23) % I2S_SPB;
		i2s_get_bits((step + 23) / I2S_SPB, p, I2S_SPB, 0, &sck, &ws, &sd);
		val |= (uint32_t)sck << 9;
		val |= (uint32_t)ws << 10;
		val |= (uint32_t)sd << 11;
	}
	/* MIPI DSI: ch12=D0N, ch13=D0P */
	{
		uint8_t d0n, d0p;
		uint32_t p = (step + 29) % MIPI_SPB;
		mipi_dsi_get_bits((step + 29) / MIPI_SPB, p, MIPI_SPB, 0, &d0n, &d0p);
		val |= (uint32_t)d0n << 12;
		val |= (uint32_t)d0p << 13;
	}
	/* SWD: ch14=SWDIO, ch15=SWCLK */
	{
		uint8_t swdio, swclk;
		uint32_t p = (step + 31) % SWD_SPB;
		swd_get_bits((step + 31) / SWD_SPB, p, SWD_SPB, 0, &swdio, &swclk);
		val |= (uint32_t)swdio << 14;
		val |= (uint32_t)swclk << 15;
	}

	/* Mirror ch0-15 to ch16-31 so all 32 channels have data. */
	val |= (val & 0xFFFF) << 16;

	return val;
}

static void logic_generator(struct sr_dev_inst *sdi, uint64_t size)
{
	struct dev_context *devc;
	uint64_t i, j;
	uint8_t pat;
	uint8_t *sample;
	const uint8_t *image_col;
	size_t col_count, col_height;
	uint64_t gray;

	devc = sdi->priv;

	switch (devc->logic_pattern) {
	case PATTERN_SIGROK:
		memset(devc->logic_data, 0x00, size);
		for (i = 0; i < size; i += devc->logic_unitsize) {
			for (j = 0; j < devc->logic_unitsize; j++) {
				pat = pattern_sigrok[(devc->step + j) % sizeof(pattern_sigrok)] >> 1;
				devc->logic_data[i + j] = ~pat;
			}
			devc->step++;
		}
		break;
	case PATTERN_RANDOM:
		demo_fill_random(&devc->prng_state, devc->logic_data, (size_t)size);
		break;
	case PATTERN_INC:
		for (i = 0; i < size; i += devc->logic_unitsize) {
			set_logic_data(devc->step, &devc->logic_data[i],
					devc->logic_unitsize);
			devc->step++;
		}
		break;
	case PATTERN_WALKING_ONE:
		/* j contains the value of the highest bit */
		j = (uint64_t)1 << (devc->num_logic_channels - 1);
		for (i = 0; i < size; i += devc->logic_unitsize) {
			set_logic_data(devc->step, &devc->logic_data[i],
					devc->logic_unitsize);
			if (devc->step == 0)
				devc->step = 1;
			else if (devc->step == j)
				devc->step = 0;
			else
				devc->step <<= 1;
		}
		break;
	case PATTERN_WALKING_ZERO:
		/* Same as walking one, only with inverted output */
		/* j contains the value of the highest bit */
		j = (uint64_t)1 << (devc->num_logic_channels - 1);
		for (i = 0; i < size; i += devc->logic_unitsize) {
			set_logic_data(~devc->step, &devc->logic_data[i],
					devc->logic_unitsize);
			if (devc->step == 0)
				devc->step = 1;
			else if (devc->step == j)
				devc->step = 0;
			else
				devc->step <<= 1;
		}
		break;
	case PATTERN_ALL_LOW:
		/* Re-fill each chunk. The memset in config_set only runs once
		 * when the pattern is selected, but the buffer is reused across
		 * calls and modified in-place by logic_fixup_feed and the PWM
		 * override. Without re-filling, stale modifications persist. */
		memset(devc->logic_data, 0x00, size);
		break;
	case PATTERN_ALL_HIGH:
		memset(devc->logic_data, 0xff, size);
		break;
	case PATTERN_SQUID:
		memset(devc->logic_data, 0x00, size);
		col_count = ARRAY_SIZE(pattern_squid);
		col_height = ARRAY_SIZE(pattern_squid[0]);
		for (i = 0; i < size; i += devc->logic_unitsize) {
			sample = &devc->logic_data[i];
			image_col = pattern_squid[devc->step];
			for (j = 0; j < devc->logic_unitsize; j++) {
				pat = image_col[j % col_height];
				sample[j] = pat;
			}
			devc->step++;
			devc->step %= col_count;
		}
		break;
	case PATTERN_GRAYCODE:
		for (i = 0; i < size; i += devc->logic_unitsize) {
			devc->step++;
			devc->step &= devc->all_logic_channels_mask;
			gray = encode_number_to_gray(devc->step);
			gray &= devc->all_logic_channels_mask;
			set_logic_data(gray, &devc->logic_data[i], devc->logic_unitsize);
		}
		break;
	case PATTERN_I2C:
		/* I2C pattern: ch0=SCL, ch1=SDA.
		 * Uses devc->step as a global bit counter. Each I2C bit
		 * occupies I2C_SPB samples. The pattern generator
		 * is a simple state machine that cycles through:
		 * START → ADDR(7+RW) → ACK → DATA(8) → ACK → ... → STOP.
		 * Address starts at 0x50 and increments each transaction.
		 * Data bytes start at 0x00 and increment within each.
		 */
		{
			const uint32_t bittime = I2C_SPB;
			uint32_t bit_phase = devc->step % bittime;
			uint32_t bit_idx = devc->step / bittime;
			uint8_t scl_bit, sda_bit;
			i2c_get_bit(bit_idx, bit_phase, bittime, &scl_bit, &sda_bit);
			uint64_t val = (uint64_t)scl_bit | ((uint64_t)sda_bit << 1);
			set_logic_data(val, &devc->logic_data[0], devc->logic_unitsize);
			for (i = devc->logic_unitsize; i < size; i += devc->logic_unitsize) {
				devc->step++;
				bit_phase = devc->step % bittime;
				bit_idx = devc->step / bittime;
				i2c_get_bit(bit_idx, bit_phase, bittime, &scl_bit, &sda_bit);
				val = (uint64_t)scl_bit | ((uint64_t)sda_bit << 1);
				set_logic_data(val, &devc->logic_data[i], devc->logic_unitsize);
			}
			devc->step++;
		}
		break;
	case PATTERN_MIXED:
		/* Mixed pattern: 32 channels of I2C+SPI+UART.
		 * Uses devc->step as a global sample counter.
		 * Each protocol has its own samples-per-bit timebase
		 * (mixed_get_sample derives bit/phase from sample_idx). */
		{
			uint32_t sample = mixed_get_sample(0, 0, 0, (uint32_t)devc->step);
			set_logic_data(sample, &devc->logic_data[0], devc->logic_unitsize);
			for (i = devc->logic_unitsize; i < size; i += devc->logic_unitsize) {
				devc->step++;
				sample = mixed_get_sample(0, 0, 0, (uint32_t)devc->step);
				set_logic_data(sample, &devc->logic_data[i], devc->logic_unitsize);
			}
			devc->step++;
		}
		break;
	default:
		sr_err("Unknown pattern: %d.", devc->logic_pattern);
		break;
	}

	/* PWM override: when enabled, PWM0 replaces channel 6's bit and
	 * PWM1 replaces channel 7's bit with a square wave generated from
	 * the user-configured frequency and duty cycle.
	 *
	 * Uses DDS (Direct Digital Synthesis) phase accumulator for precise
	 * frequency and duty cycle. The 32-bit phase provides sub-sample
	 * resolution, avoiding truncation errors that occur when samplerate
	 * is not an exact multiple of the PWM frequency (which caused
	 * non-uniform waveforms with the old integer-division approach). */
	if (devc->pwm0_en || devc->pwm1_en) {
		uint32_t pwm0_step = 0, pwm0_thresh = 0;
		uint32_t pwm1_step = 0, pwm1_thresh = 0;

		if (devc->pwm0_en && devc->pwm0_freq > 0 && devc->cur_samplerate > 0) {
			double step_d = (double)devc->pwm0_freq * 4294967296.0
					/ (double)devc->cur_samplerate;
			if (step_d < 4294967296.0)
				pwm0_step = (uint32_t)step_d;
			pwm0_thresh = (devc->pwm0_duty >= 100.0) ? 0xFFFFFFFF :
				      (devc->pwm0_duty <= 0.0) ? 0 :
				      (uint32_t)(devc->pwm0_duty * 4294967296.0 / 100.0);
		}
		if (devc->pwm1_en && devc->pwm1_freq > 0 && devc->cur_samplerate > 0) {
			double step_d = (double)devc->pwm1_freq * 4294967296.0
					/ (double)devc->cur_samplerate;
			if (step_d < 4294967296.0)
				pwm1_step = (uint32_t)step_d;
			pwm1_thresh = (devc->pwm1_duty >= 100.0) ? 0xFFFFFFFF :
				      (devc->pwm1_duty <= 0.0) ? 0 :
				      (uint32_t)(devc->pwm1_duty * 4294967296.0 / 100.0);
		}

		uint64_t sample_index;
		for (i = 0; i < size; i += devc->logic_unitsize) {
			/* Absolute sample index = sent_samples + (this sample offset). */
			sample_index = devc->sent_samples + (i / devc->logic_unitsize);
			if (pwm0_step > 0) {
				/* DDS phase: (sample_index * step) mod 2^32 */
				uint32_t phase = (uint32_t)(sample_index * (uint64_t)pwm0_step);
				uint8_t bit_val = (phase < pwm0_thresh) ? 0x40 : 0x00;
				devc->logic_data[i] = (devc->logic_data[i] & ~0x40) | bit_val;
			}
			if (pwm1_step > 0) {
				uint32_t phase = (uint32_t)(sample_index * (uint64_t)pwm1_step);
				uint8_t bit_val = (phase < pwm1_thresh) ? 0x80 : 0x00;
				devc->logic_data[i] = (devc->logic_data[i] & ~0x80) | bit_val;
			}
		}
	}
}

/*
 * Fixup a memory image of generated logic data before it gets sent to
 * the session's datafeed. Mask out content from disabled channels.
 *
 * TODO: Need we apply a channel map, and enforce a dense representation
 * of the enabled channels' data?
 */
static void logic_fixup_feed(struct dev_context *devc,
		struct sr_datafeed_logic *logic)
{
	size_t fp_off;
	uint8_t fp_mask;
	size_t off, idx;
	uint8_t *sample;

	fp_off = devc->first_partial_logic_index;
	fp_mask = devc->first_partial_logic_mask;
	if (fp_off == logic->unitsize)
		return;

	for (off = 0; off < logic->length; off += logic->unitsize) {
		sample = logic->data + off;
		sample[fp_off] &= fp_mask;
		for (idx = fp_off + 1; idx < logic->unitsize; idx++)
			sample[idx] = 0x00;
	}
}

/* Pack 8 sample values into 1 byte per channel in LA_CROSS_DATA format.
 * For each channel ch, extracts bit ch from each of 8 samples and packs
 * them into a single byte: bit b = sample b's channel ch value.
 * group points to the start of a 64-sample group (num_channels * 8 bytes).
 * s8 is the byte index within each channel's 8-byte block (0-7). */
static inline void pack_8_cross(uint8_t *group, int s8,
	const uint64_t vals[8], const int *enabled_ch, int num_channels)
{
	for (int k = 0; k < num_channels; k++) {
		int ch = enabled_ch[k];           /* real channel index for tight slot k */
		uint64_t mask = 1ULL << ch;
		group[k * 8 + s8] = (uint8_t)(
			(!!(vals[0] & mask))       |
			(!!(vals[1] & mask)) << 1  |
			(!!(vals[2] & mask)) << 2  |
			(!!(vals[3] & mask)) << 3  |
			(!!(vals[4] & mask)) << 4  |
			(!!(vals[5] & mask)) << 5  |
			(!!(vals[6] & mask)) << 6  |
			(!!(vals[7] & mask)) << 7);
	}
}

/* Generate logic data directly in LA_CROSS_DATA channel-block format.
 *
 * This is the unified fast path for ALL pattern types. It eliminates three
 * passes (logic_generator → logic_fixup_feed → convert_to_cross_data) and
 * replaces them with a single pass that writes directly to cross_data_buf.
 *
 * For each group of 64 samples, each channel gets 8 bytes (64 bits packed
 * 8 per byte). The function handles:
 *   - All 9 pattern types (SIGROK, RANDOM, INC, WALKING_ONE/ZERO,
 *     ALL_LOW/HIGH, SQUID, GRAYCODE)
 *   - Disabled channel masking (fixup) in cross domain
 *   - PWM override (channels 6/7) in cross domain
 *
 * start_sample_index is the absolute sample index of the first sample in
 * this batch (= devc->sent_samples + logic_done), needed for PWM phase. */
static void logic_generator_cross(struct sr_dev_inst *sdi,
	uint64_t num_samples, uint64_t start_sample_index)
{
	struct dev_context *devc = sdi->priv;
	size_t unitsize = devc->logic_unitsize;
	/* Tight packing: only enabled logic channels occupy the stream, in
	 * ascending channel-index order (matches PXView's _ch_index fill order
	 * and real PXLogic hardware layout). If a channel is disabled it is
	 * NOT present, so no per-disabled-channel zeroing is needed. */
	int tight_index[32];
	int enabled_ch[32];
	int num_channels = 0;
	int nn;
	memset(tight_index, -1, sizeof(tight_index));
	for (GSList *l = sdi->channels; l; l = l->next) {
		struct sr_channel *ch = l->data;
		if (ch && ch->type == SR_CHANNEL_LOGIC && ch->enabled &&
			ch->index >= 0 && ch->index < 32) {
			nn = num_channels++;
			tight_index[ch->index] = nn;
			enabled_ch[nn] = ch->index;
		}
	}
	uint64_t num_groups = num_samples / 64;
	uint8_t *dst = devc->cross_data_buf;
	size_t group_size = (size_t)num_channels * 8;

	switch (devc->logic_pattern) {
	case PATTERN_RANDOM:
		demo_fill_random(&devc->prng_state, dst,
			num_groups * group_size);
		break;

	case PATTERN_ALL_LOW:
		memset(dst, 0, num_groups * group_size);
		break;

	case PATTERN_ALL_HIGH:
		memset(dst, 0xff, num_groups * group_size);
		break;

	case PATTERN_SIGROK:
		for (uint64_t g = 0; g < num_groups; g++) {
			uint8_t *group = dst + g * group_size;
			for (int s8 = 0; s8 < 8; s8++) {
				uint64_t vals[8];
				for (int b = 0; b < 8; b++) {
					uint64_t step = devc->step;
					vals[b] = 0;
					for (size_t j = 0; j < unitsize; j++) {
						uint8_t pat = pattern_sigrok[
							(step + j) % sizeof(pattern_sigrok)] >> 1;
						vals[b] |= ((uint64_t)(uint8_t)~pat) << (j * 8);
					}
					devc->step++;
				}
				pack_8_cross(group, s8, vals, enabled_ch, num_channels);
			}
		}
		break;

	case PATTERN_INC:
		for (uint64_t g = 0; g < num_groups; g++) {
			uint8_t *group = dst + g * group_size;
			for (int s8 = 0; s8 < 8; s8++) {
				uint64_t vals[8];
				vals[0] = devc->step++;
				vals[1] = devc->step++;
				vals[2] = devc->step++;
				vals[3] = devc->step++;
				vals[4] = devc->step++;
				vals[5] = devc->step++;
				vals[6] = devc->step++;
				vals[7] = devc->step++;
				pack_8_cross(group, s8, vals, enabled_ch, num_channels);
			}
		}
		break;

	case PATTERN_WALKING_ONE: {
		uint64_t high_bit = (uint64_t)1 << (devc->num_logic_channels - 1);
		for (uint64_t g = 0; g < num_groups; g++) {
			uint8_t *group = dst + g * group_size;
			for (int s8 = 0; s8 < 8; s8++) {
				uint64_t vals[8];
				for (int b = 0; b < 8; b++) {
					vals[b] = devc->step;
					if (devc->step == 0)
						devc->step = 1;
					else if (devc->step == high_bit)
						devc->step = 0;
					else
						devc->step <<= 1;
				}
				pack_8_cross(group, s8, vals, enabled_ch, num_channels);
			}
		}
		break;
	}

	case PATTERN_WALKING_ZERO: {
		uint64_t high_bit = (uint64_t)1 << (devc->num_logic_channels - 1);
		for (uint64_t g = 0; g < num_groups; g++) {
			uint8_t *group = dst + g * group_size;
			for (int s8 = 0; s8 < 8; s8++) {
				uint64_t vals[8];
				for (int b = 0; b < 8; b++) {
					vals[b] = ~devc->step;
					if (devc->step == 0)
						devc->step = 1;
					else if (devc->step == high_bit)
						devc->step = 0;
					else
						devc->step <<= 1;
				}
				pack_8_cross(group, s8, vals, enabled_ch, num_channels);
			}
		}
		break;
	}

	case PATTERN_SQUID: {
		size_t col_count = ARRAY_SIZE(pattern_squid);
		size_t col_height = ARRAY_SIZE(pattern_squid[0]);
		for (uint64_t g = 0; g < num_groups; g++) {
			uint8_t *group = dst + g * group_size;
			for (int s8 = 0; s8 < 8; s8++) {
				uint64_t vals[8];
				for (int b = 0; b < 8; b++) {
					const uint8_t *image_col = pattern_squid[devc->step];
					vals[b] = 0;
					for (size_t j = 0; j < unitsize; j++)
						vals[b] |= ((uint64_t)image_col[j % col_height]) << (j * 8);
					devc->step++;
					devc->step %= col_count;
				}
				pack_8_cross(group, s8, vals, enabled_ch, num_channels);
			}
		}
		break;
	}

	case PATTERN_GRAYCODE:
		for (uint64_t g = 0; g < num_groups; g++) {
			uint8_t *group = dst + g * group_size;
			for (int s8 = 0; s8 < 8; s8++) {
				uint64_t vals[8];
				for (int b = 0; b < 8; b++) {
					devc->step++;
					devc->step &= devc->all_logic_channels_mask;
					uint64_t gray = devc->step ^ (devc->step >> 1);
					vals[b] = gray & devc->all_logic_channels_mask;
				}
				pack_8_cross(group, s8, vals, enabled_ch, num_channels);
			}
		}
	break;

	case PATTERN_I2C:
		for (uint64_t g = 0; g < num_groups; g++) {
			uint8_t *group = dst + g * group_size;
			for (int s8 = 0; s8 < 8; s8++) {
				uint64_t vals[8];
				for (int b = 0; b < 8; b++) {
					uint32_t bit_phase = devc->step % I2C_SPB;
					uint32_t bit_idx = devc->step / I2C_SPB;
					uint8_t scl_bit, sda_bit;
					i2c_get_bit(bit_idx, bit_phase,
						I2C_SPB, &scl_bit, &sda_bit);
					vals[b] = (uint64_t)scl_bit
						| ((uint64_t)sda_bit << 1);
					devc->step++;
				}
				pack_8_cross(group, s8, vals, enabled_ch, num_channels);
			}
		}
		break;

	case PATTERN_MIXED:
		for (uint64_t g = 0; g < num_groups; g++) {
			uint8_t *group = dst + g * group_size;
			for (int s8 = 0; s8 < 8; s8++) {
				uint64_t vals[8];
				for (int b = 0; b < 8; b++) {
					/* mixed_get_sample derives each bus's bit/phase
					 * from sample_idx internally (independent SPB). */
					vals[b] = mixed_get_sample(0, 0, 0,
						(uint32_t)devc->step);
					devc->step++;
				}
				pack_8_cross(group, s8, vals, enabled_ch, num_channels);
			}
		}
		break;

	default:
		memset(dst, 0, num_groups * group_size);
		sr_err("Unknown pattern: %d.", devc->logic_pattern);
		break;
	}

	/* Apply PWM override in cross domain.
	 * PWM0 replaces channel 6's bit, PWM1 replaces channel 7's bit.
	 * In cross format, channel ch's data is at offset ch*8 per group,
	 * 8 bytes = 64 packed bits. We recompute the channel's 8 bytes
	 * using the DDS phase accumulator, matching logic_generator's PWM. */
	if (devc->pwm0_en || devc->pwm1_en) {
		uint32_t pwm0_step = 0, pwm0_thresh = 0;
		uint32_t pwm1_step = 0, pwm1_thresh = 0;

		if (devc->pwm0_en && devc->pwm0_freq > 0 &&
				devc->cur_samplerate > 0) {
			double step_d = (double)devc->pwm0_freq * 4294967296.0
					/ (double)devc->cur_samplerate;
			if (step_d < 4294967296.0)
				pwm0_step = (uint32_t)step_d;
			pwm0_thresh = (devc->pwm0_duty >= 100.0) ? 0xFFFFFFFF :
				      (devc->pwm0_duty <= 0.0) ? 0 :
				      (uint32_t)(devc->pwm0_duty * 4294967296.0 / 100.0);
		}
		if (devc->pwm1_en && devc->pwm1_freq > 0 &&
				devc->cur_samplerate > 0) {
			double step_d = (double)devc->pwm1_freq * 4294967296.0
					/ (double)devc->cur_samplerate;
			if (step_d < 4294967296.0)
				pwm1_step = (uint32_t)step_d;
			pwm1_thresh = (devc->pwm1_duty >= 100.0) ? 0xFFFFFFFF :
				      (devc->pwm1_duty <= 0.0) ? 0 :
				      (uint32_t)(devc->pwm1_duty * 4294967296.0 / 100.0);
		}

		for (uint64_t g = 0; g < num_groups; g++) {
			uint8_t *group = dst + g * group_size;
			int s6, s7;
			if (pwm0_step > 0 && (s6 = tight_index[6]) >= 0) {
				uint8_t *ch6 = group + s6 * 8;
				for (int s8 = 0; s8 < 8; s8++) {
					uint8_t byte = 0;
					for (int b = 0; b < 8; b++) {
						uint64_t idx = start_sample_index
							+ g * 64 + (uint64_t)s8 * 8 + (uint64_t)b;
						uint32_t phase = (uint32_t)(idx * (uint64_t)pwm0_step);
						if (phase < pwm0_thresh)
							byte |= (1u << b);
					}
					ch6[s8] = byte;
				}
			}
			if (pwm1_step > 0 && (s7 = tight_index[7]) >= 0) {
				uint8_t *ch7 = group + s7 * 8;
				for (int s8 = 0; s8 < 8; s8++) {
					uint8_t byte = 0;
					for (int b = 0; b < 8; b++) {
						uint64_t idx = start_sample_index
							+ g * 64 + (uint64_t)s8 * 8 + (uint64_t)b;
						uint32_t phase = (uint32_t)(idx * (uint64_t)pwm1_step);
						if (phase < pwm1_thresh)
							byte |= (1u << b);
					}
					ch7[s8] = byte;
				}
			}
		}
	}
}

/*
 * Convert sample-interleaved logic data to LA_CROSS_DATA (channel-block)
 * format, matching pxlogic's hardware DMA layout.
 *
 * Input (sample-interleaved): each sample is unitsize bytes, where bit k
 *   of byte b represents channel (b*8 + k).
 * Output (LA_CROSS_DATA): groups of 64 samples per channel, 8 bytes per
 *   channel (64 samples packed 8 per byte), channels in sequence:
 *   [ch0_8bytes][ch1_8bytes]...[chN-1_8bytes] per group.
 *
 * num_samples must be a multiple of 64. Output size = (num_samples/64) *
 * num_channels * 8 (tight: only the num_channels enabled logic channels —
 * NOT num_samples * unitsize).
 */
static void convert_to_cross_data(const uint8_t *src, uint8_t *dst,
	uint64_t num_samples, size_t unitsize, int num_channels)
{
	uint64_t num_groups = num_samples / 64;

	memset(dst, 0, num_groups * num_channels * 8);

	for (uint64_t g = 0; g < num_groups; g++) {
		const uint8_t *src_g = src + (g * 64) * unitsize;
		uint8_t *dst_g = dst + g * num_channels * 8;
		for (int ch = 0; ch < num_channels; ch++) {
			size_t bp = (size_t)(ch >> 3);
			uint8_t mask = (uint8_t)(1u << (ch & 7));
			uint8_t *out = dst_g + ch * 8;
			for (uint64_t s8 = 0; s8 < 8; s8++) {
				uint8_t ob = 0;
				const uint8_t *p = src_g + (s8 * 8) * unitsize + bp;
				for (uint64_t b = 0; b < 8; b++) {
					if (p[b * unitsize] & mask)
						ob |= (uint8_t)(1u << b);
				}
				out[s8] = ob;
			}
		}
	}
}

/*
 * 对生成的模拟样本施加耦合效果 (模拟真实示波器模拟前端的 AC/DC/GND 耦合)。
 *
 * DSL 等真实硬件驱动通过 I2C 配置耦合继电器, 由硬件物理实现; demo 没有
 * 硬件前端, 必须在软件层模拟。行为参考示波器耦合原理:
 *   GND (0): 信号接地, 输出恒 0 (观察基线噪声/零点)
 *   DC  (1): 直通, 信号原样通过 (保留直流分量)
 *   AC  (2): 高通滤波, 去除直流分量 (一阶 RC, 截止频率 ~ 采样率/200)
 *
 * AC 耦合一阶 RC 高通: y[n] = a*(y[n-1] + x[n] - x[n-1])
 *   a = RC/(RC+dt), dt = 1/samplerate, RC 选 ~200 个样本周期使低频被抑制。
 */
#define DEMO_AC_COUPLING_TAU 200.0f  /* RC 时间常数 (样本数) */

static void apply_analog_coupling(struct analog_gen *ag,
		struct dev_context *devc, float *data, unsigned int count)
{
	uint8_t coupling;
	int aidx;

	if (!ag->ch || !devc)
		return;

	/* 计算模拟通道索引 (与 api.c config_get/set 的 aidx 一致)。 */
	aidx = ag->ch->index - devc->num_logic_channels;
	if (aidx < 0 || aidx >= devc->num_analog_channels || aidx >= DSO_MAX_CHANNELS)
		return;

	coupling = devc->analog_coupling[aidx];

	if (coupling == 1 /* SR_DC_COUPLING */)
		return;  /* DC: 直通, 无处理 */

	if (coupling == 0 /* SR_GND_COUPLING */) {
		/* GND: 强制所有样本为 0。 */
		for (unsigned int i = 0; i < count; i++)
			data[i] = 0.0f;
		/* 重置 AC 滤波器状态避免切换后跳变。 */
		ag->ac_prev_input = 0.0f;
		ag->ac_prev_output = 0.0f;
		return;
	}

	if (coupling == 2 /* SR_AC_COUPLING */) {
		/* 一阶 RC 高通滤波器: y[n] = a*(y[n-1] + x[n] - x[n-1]) */
		const float a = DEMO_AC_COUPLING_TAU / (DEMO_AC_COUPLING_TAU + 1.0f);
		float prev_in = ag->ac_prev_input;
		float prev_out = ag->ac_prev_output;
		for (unsigned int i = 0; i < count; i++) {
			float x = data[i];
			float y = a * (prev_out + x - prev_in);
			data[i] = y;
			prev_in = x;
			prev_out = y;
		}
		ag->ac_prev_input = prev_in;
		ag->ac_prev_output = prev_out;
		return;
	}

	/* 未知耦合值, 按 DC 处理 (直通)。 */
}

/*
 * 对 DSO 样本施加耦合效果。与 apply_analog_coupling 行为一致, 但针对
 * DSO 的 byte-interleaved 缓冲区布局 [ch0_s0, ch1_s0, ch0_s1, ...] 操作。
 *
 * DSO 样本是 8-bit (0..255), 中心值 mid=128 (DSO_SAMPLE_BITS/2 对应的 ADC
 * 中点)。各耦合模式:
 *   GND (0): 样本强制为 mid (基线), 观察零点
 *   DC  (1): 直通
 *   AC  (2): 一阶 RC 高通 (去除直流分量), 围绕 mid 振荡
 *
 * 注意: 此函数必须在 demo_dso_vdiv_scale() 之后调用 (vdiv 缩放改变样本幅度,
 * 耦合应在最终样本值上应用)。每通道 AC 滤波状态保存在
 * devc->dso_ac_prev_input/output, 跨包连续。
 */
static void apply_dso_coupling(struct dev_context *devc, uint8_t *data,
		uint64_t sample_count, uint8_t en_ch_num)
{
	const uint8_t mid = 128;
	const float a = DEMO_AC_COUPLING_TAU / (DEMO_AC_COUPLING_TAU + 1.0f);

	if (!devc || !data || en_ch_num == 0)
		return;

	for (uint8_t ch = 0; ch < en_ch_num && ch < DSO_MAX_CHANNELS; ch++) {
		uint8_t coupling = devc->dso_coupling[ch];

		if (coupling == 1 /* SR_DC_COUPLING */)
			continue;  /* DC: 直通 */

		if (coupling == 0 /* SR_GND_COUPLING */) {
			/* GND: 强制所有样本为 mid (基线)。 */
			for (uint64_t i = 0; i < sample_count; i++)
				data[i * en_ch_num + ch] = mid;
			/* 重置 AC 滤波器状态避免切换后跳变。 */
			devc->dso_ac_prev_input[ch] = 0.0f;
			devc->dso_ac_prev_output[ch] = 0.0f;
			continue;
		}

		if (coupling == 2 /* SR_AC_COUPLING */) {
			/* 一阶 RC 高通: 对 (x - mid) 做高通, 输出加回 mid。
			 * y' = a*(y'[n-1] + (x-mid) - (x_prev-mid))
			 * y  = y' + mid */
			float prev_in = devc->dso_ac_prev_input[ch];
			float prev_out = devc->dso_ac_prev_output[ch];
			for (uint64_t i = 0; i < sample_count; i++) {
				float x = (float)data[i * en_ch_num + ch] - mid;
				float y = a * (prev_out + x - prev_in);
				int v = (int)(y + mid + 0.5f);
				if (v < 0) v = 0;
				if (v > 255) v = 255;
				data[i * en_ch_num + ch] = (uint8_t)v;
				prev_in = x;
				prev_out = y;
			}
			devc->dso_ac_prev_input[ch] = prev_in;
			devc->dso_ac_prev_output[ch] = prev_out;
			continue;
		}
		/* 未知耦合值, 按 DC 处理 (直通)。 */
	}
}

static void send_analog_packet(struct analog_gen *ag,
		struct sr_dev_inst *sdi, uint64_t *analog_sent,
		uint64_t analog_pos, uint64_t analog_todo)
{
	struct sr_datafeed_packet packet;
	struct dev_context *devc;
	struct analog_pattern *pattern;
	uint64_t sending_now, to_avg;
	int ag_pattern_pos;
	unsigned int i;
	float amplitude, offset, value;
	float *data;

	if (!ag->ch || !ag->ch->enabled)
		return;

	devc = sdi->priv;
	packet.type = SR_DF_ANALOG;
	packet.payload = &ag->packet;

	pattern = devc->analog_patterns[ag->pattern];

	ag->packet.meaning->channels = g_slist_append(NULL, ag->ch);
	ag->packet.meaning->mq = ag->mq;
	ag->packet.meaning->mqflags = ag->mq_flags;

	/* Set a unit for the given quantity. */
	if (ag->mq == SR_MQ_VOLTAGE)
		ag->packet.meaning->unit = SR_UNIT_VOLT;
	else if (ag->mq == SR_MQ_CURRENT)
		ag->packet.meaning->unit = SR_UNIT_AMPERE;
	else if (ag->mq == SR_MQ_RESISTANCE)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_CAPACITANCE)
		ag->packet.meaning->unit = SR_UNIT_FARAD;
	else if (ag->mq == SR_MQ_TEMPERATURE)
		ag->packet.meaning->unit = SR_UNIT_CELSIUS;
	else if (ag->mq == SR_MQ_FREQUENCY)
		ag->packet.meaning->unit = SR_UNIT_HERTZ;
	else if (ag->mq == SR_MQ_DUTY_CYCLE)
		ag->packet.meaning->unit = SR_UNIT_PERCENTAGE;
	else if (ag->mq == SR_MQ_CONTINUITY)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_PULSE_WIDTH)
		ag->packet.meaning->unit = SR_UNIT_PERCENTAGE;
	else if (ag->mq == SR_MQ_CONDUCTANCE)
		ag->packet.meaning->unit = SR_UNIT_SIEMENS;
	else if (ag->mq == SR_MQ_POWER)
		ag->packet.meaning->unit = SR_UNIT_WATT;
	else if (ag->mq == SR_MQ_GAIN)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_SOUND_PRESSURE_LEVEL)
		ag->packet.meaning->unit = SR_UNIT_DECIBEL_SPL;
	else if (ag->mq == SR_MQ_CARBON_MONOXIDE)
		ag->packet.meaning->unit = SR_UNIT_CONCENTRATION;
	else if (ag->mq == SR_MQ_RELATIVE_HUMIDITY)
		ag->packet.meaning->unit = SR_UNIT_HUMIDITY_293K;
	else if (ag->mq == SR_MQ_TIME)
		ag->packet.meaning->unit = SR_UNIT_SECOND;
	else if (ag->mq == SR_MQ_WIND_SPEED)
		ag->packet.meaning->unit = SR_UNIT_METER_SECOND;
	else if (ag->mq == SR_MQ_PRESSURE)
		ag->packet.meaning->unit = SR_UNIT_HECTOPASCAL;
	else if (ag->mq == SR_MQ_PARALLEL_INDUCTANCE)
		ag->packet.meaning->unit = SR_UNIT_HENRY;
	else if (ag->mq == SR_MQ_PARALLEL_CAPACITANCE)
		ag->packet.meaning->unit = SR_UNIT_FARAD;
	else if (ag->mq == SR_MQ_PARALLEL_RESISTANCE)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_SERIES_INDUCTANCE)
		ag->packet.meaning->unit = SR_UNIT_HENRY;
	else if (ag->mq == SR_MQ_SERIES_CAPACITANCE)
		ag->packet.meaning->unit = SR_UNIT_FARAD;
	else if (ag->mq == SR_MQ_SERIES_RESISTANCE)
		ag->packet.meaning->unit = SR_UNIT_OHM;
	else if (ag->mq == SR_MQ_DISSIPATION_FACTOR)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_QUALITY_FACTOR)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_PHASE_ANGLE)
		ag->packet.meaning->unit = SR_UNIT_DEGREE;
	else if (ag->mq == SR_MQ_DIFFERENCE)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_COUNT)
		ag->packet.meaning->unit = SR_UNIT_PIECE;
	else if (ag->mq == SR_MQ_POWER_FACTOR)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else if (ag->mq == SR_MQ_APPARENT_POWER)
		ag->packet.meaning->unit = SR_UNIT_VOLT_AMPERE;
	else if (ag->mq == SR_MQ_MASS)
		ag->packet.meaning->unit = SR_UNIT_GRAM;
	else if (ag->mq == SR_MQ_HARMONIC_RATIO)
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;
	else
		ag->packet.meaning->unit = SR_UNIT_UNITLESS;

	if (!devc->avg) {
		ag_pattern_pos = analog_pos % pattern->num_samples;
		sending_now = MIN(analog_todo, pattern->num_samples - ag_pattern_pos);
		/* 判断当前通道耦合是否为 DC (直通)。非 DC 时即使 amplitude/offset
		 * 未变也必须走慢速路径 (需修改样本), 不能直接指向 pattern->data。 */
		int aidx_coupling = ag->ch ? ag->ch->index - devc->num_logic_channels : -1;
		uint8_t cur_coupling = (aidx_coupling >= 0
				&& aidx_coupling < devc->num_analog_channels
				&& aidx_coupling < DSO_MAX_CHANNELS)
			? devc->analog_coupling[aidx_coupling] : 1 /* DC */;
		gboolean need_coupling = (cur_coupling != 1 /* DC */);
		if (ag->amplitude != DEFAULT_ANALOG_AMPLITUDE ||
			ag->offset != DEFAULT_ANALOG_OFFSET ||
			ag->pattern == PATTERN_ANALOG_RANDOM || need_coupling) {
			/*
			 * Amplitude or offset changed (or we are generating
			 * random data, or coupling != DC needs per-sample
			 * processing), modify each sample.
			 */
			if (ag->pattern == PATTERN_ANALOG_RANDOM) {
				amplitude = ag->amplitude / 500.0;
				offset = ag->offset - DEFAULT_ANALOG_OFFSET - ag->amplitude;
			} else {
				amplitude = ag->amplitude / DEFAULT_ANALOG_AMPLITUDE;
				offset = ag->offset - DEFAULT_ANALOG_OFFSET;
			}
			/* 耦合处理时用 coupling_buf, 避免污染共享的 pattern->data
			 * 或上一包的 packet.data。DC 耦合保持原逻辑用 packet.data。 */
			data = need_coupling ? ag->coupling_buf : ag->packet.data;
			for (i = 0; i < sending_now; i++) {
				if (ag->pattern == PATTERN_ANALOG_RANDOM) {
					/* Use pre-filled cyclic buffer for stable random
					 * data (repeats like a real captured signal). */
					if (devc->analog_random_buf)
						data[i] = (devc->analog_random_buf[
							(analog_pos + i) % devc->analog_random_buf_len]
							/ 255.0 * 1000) * amplitude + offset;
					else
						data[i] = (rand() % 1000) * amplitude + offset;
				} else
					data[i] = pattern->data[ag_pattern_pos + i] * amplitude + offset;
			}
			/* 应用耦合效果 (GND 置零 / AC 高通; DC 在 apply 内直通返回)。 */
			apply_analog_coupling(ag, devc, data, sending_now);
			ag->packet.data = data;
		} else {
			/* Amplitude and offset unchanged, use the fast way. */
			ag->packet.data = pattern->data + ag_pattern_pos;
		}
		ag->packet.num_samples = sending_now;
		sr_session_send(sdi, &packet);

		/* Whichever channel group gets there first. */
		*analog_sent = MAX(*analog_sent, sending_now);
	} else {
		ag_pattern_pos = analog_pos % pattern->num_samples;
		to_avg = MIN(analog_todo, pattern->num_samples - ag_pattern_pos);
		if (ag->pattern == PATTERN_ANALOG_RANDOM) {
			amplitude = ag->amplitude / 500.0;
			offset = ag->offset - DEFAULT_ANALOG_OFFSET - ag->amplitude;
		} else {
			amplitude = ag->amplitude / DEFAULT_ANALOG_AMPLITUDE;
			offset = ag->offset - DEFAULT_ANALOG_OFFSET;
		}

		for (i = 0; i < to_avg; i++) {
			if (ag->pattern == PATTERN_ANALOG_RANDOM)
				value = (rand() % 1000) * amplitude + offset;
			else
				value = *(pattern->data + ag_pattern_pos + i) * amplitude + offset;
			ag->avg_val = (ag->avg_val + value) / 2;
			ag->num_avgs++;
			/* Time to send averaged data? */
			if ((devc->avg_samples > 0) && (ag->num_avgs >= devc->avg_samples))
				goto do_send;
		}

		if (devc->avg_samples == 0) {
			/*
			 * We're averaging all the samples, so wait with
			 * sending until the very end.
			 */
			*analog_sent = ag->num_avgs;
			return;
		}

do_send:
		/* 对平均值也应用耦合 (avg 模式下每包只发 1 个样本, GND 直接置零,
		 * AC 高通对单点退化为减去直流估计 — 用 prev_output 逼近)。 */
		apply_analog_coupling(ag, devc, &ag->avg_val, 1);
		ag->packet.data = &ag->avg_val;
		ag->packet.num_samples = 1;

		sr_session_send(sdi, &packet);
		*analog_sent = ag->num_avgs;

		ag->num_avgs = 0;
		ag->avg_val = 0.0f;
	}
}

/* Clear DSO config-change flags. demo_send_dso_packet() regenerates the
 * waveform on every call from current dso_pattern/vdiv/offset/timebase, so
 * the flags only need clearing to mark "regeneration consumed". Mirrors old
 * fork demo's dso_wavelength_updata() contract: config_set sets a flag, the
 * next packet send regenerates and clears it. */
SR_PRIV void dso_wavelength_updata(struct dev_context *devc)
{
	if (!devc)
		return;
	devc->dso_vdiv_change = FALSE;
	devc->dso_offset_change = FALSE;
	devc->dso_timebase_change = FALSE;
}

/* Pre-fill the analog random cyclic buffer. Called once at acquisition start
 * so PATTERN_ANALOG_RANDOM streams a stable, repeating noise pattern (like a
 * real captured signal) instead of fresh random noise every packet. Mirrors
 * old fork demo's init_analog_random_data(). */
SR_PRIV int init_analog_random_data(struct dev_context *devc)
{
	size_t i;

	if (!devc)
		return SR_ERR_ARG;
	if (!devc->analog_random_buf) {
		devc->analog_random_buf = g_malloc(ANALOG_RANDOM_BUF_LEN);
		if (!devc->analog_random_buf)
			return SR_ERR_MALLOC;
		devc->analog_random_buf_len = ANALOG_RANDOM_BUF_LEN;
	}
	for (i = 0; i < devc->analog_random_buf_len; i++)
		devc->analog_random_buf[i] = (uint8_t)(rand() & 0xff);
	devc->analog_random_read_pos = 0;
	return SR_OK;
}

/* Generate one 8-bit DSO sample for the given pattern at sample index i.
 * wavelength = samples per period. mid/amp define the vertical range.
 * ch_idx adds a small phase offset per channel so traces don't overlap. */
static uint8_t demo_dso_sample(enum demo_dso_pattern pat, uint64_t i,
		uint8_t mid, uint8_t amp, int ch_idx, uint64_t wavelength)
{
	uint64_t pos;
	double val;

	switch (pat) {
	case DEMO_DSO_PATTERN_RANDOM:
		return (uint8_t)((rand() % 120) + 68);
	case DEMO_DSO_PATTERN_SINE:
		/* Per-channel phase offset so channels don't overlap. */
		val = mid + amp * sin(2.0 * M_PI * (i + ch_idx * (wavelength / 4))
				/ (double)wavelength);
		if (val < 0) val = 0;
		if (val > 255) val = 255;
		return (uint8_t)val;
	case DEMO_DSO_PATTERN_SQUARE:
		pos = (i + ch_idx * (wavelength / 2)) % wavelength;
		return (pos < wavelength / 2) ? (uint8_t)(mid + amp) : (uint8_t)(mid - amp);
	case DEMO_DSO_PATTERN_SAWTOOTH:
		pos = (i + ch_idx * (wavelength / 4)) % wavelength;
		return (uint8_t)(mid - amp + (uint16_t)(2 * amp * pos / wavelength));
	case DEMO_DSO_PATTERN_TRIANGLE:
		pos = (i + ch_idx * (wavelength / 4)) % wavelength;
		if (pos < wavelength / 2)
			return (uint8_t)(mid - amp
				+ (uint16_t)(2 * amp * pos / (wavelength / 2)));
		else
			return (uint8_t)(mid + amp
				- (uint16_t)(2 * amp * (pos - wavelength / 2) / (wavelength / 2)));
	default:
		return mid;
	}
}

/*
 * Send one DSO frame (one SR_DF_DSO packet) for the demo device.
 *
 * Generates waveform data based on devc->dso_pattern (random/sine/square/
 * sawtooth/triangle) for each enabled DSO channel, interleaved as
 * [ch0_s0, ch1_s0, ch0_s1, ch1_s1, ...] which matches the layout expected
 * by DsoSnapshot::append_data(). Per-channel vdiv/offset/trig_value apply
 * vertical shifts so the traces look distinct.
 *
 * In instant mode, sends a progressive slice based on elapsed time rather
 * than the full frame, so the GUI shows a live scrolling waveform.
 * After sending the packet the acquisition is stopped — DSO mode delivers
 * one complete frame per acquisition (single-shot). The caller may restart
 * acquisition for the next frame (repeat/loop mode handled in
 * dev_acquisition_stop).
 */
SR_PRIV int demo_send_dso_packet(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_dso dso;
	struct sr_channel *ch;
	GSList *l;
	uint8_t en_ch_num;
	uint8_t *p;
	uint64_t i;
	uint64_t sending_samples;
	uint64_t wavelength;
	uint8_t mid, amp;
	int ch_idx;

	if (!sdi || !sdi->priv)
		return SR_ERR_ARG;

	devc = sdi->priv;

	/* Consume any pending config-change flags (regeneration is implicit
	 * — we build the waveform from current config on every call). */
	dso_wavelength_updata(devc);

	/* Count enabled DSO channels. */
	en_ch_num = 0;
	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (ch && ch->type == SR_CHANNEL_DSO && ch->enabled)
			en_ch_num++;
	}
	if (en_ch_num == 0)
		return SR_ERR;

	/* In instant mode, send a progressive slice based on how many samples
	 * have already been sent, so the GUI shows a live scrolling waveform.
	 * Otherwise send the full frame (single-shot DSO). */
	sending_samples = DSO_PACKET_LEN;
	if (devc->instant && devc->dso_sent_samples > 0) {
		/* Send a fraction of the remaining frame each tick. */
		uint64_t remaining = (devc->dso_sent_samples >= DSO_PACKET_LEN)
			? 0 : (DSO_PACKET_LEN - devc->dso_sent_samples);
		if (remaining > 0)
			sending_samples = MIN(remaining, DSO_PACKET_LEN / 10);
		else
			sending_samples = DSO_PACKET_LEN; /* wrap: start a new frame */
	}

	/* (Re)allocate the DSO buffer if needed (full-frame sized). */
	if (!devc->dso_buf) {
		devc->dso_buf = g_malloc(DSO_PACKET_LEN * en_ch_num);
		if (!devc->dso_buf)
			return SR_ERR_MALLOC;
	}

	/* Waveform geometry: ~4 periods across the screen. mid=128 (8-bit
	 * center). amp scales with the first enabled channel's vdiv so
	 * changing vdiv visibly changes amplitude. */
	wavelength = DSO_PACKET_LEN / 4;
	if (wavelength < 20)
		wavelength = 20;
	mid = 128;
	amp = 60;

	/*
	 * Fill interleaved samples using the selected DSO pattern. Each
	 * channel gets a phase offset (via ch_idx) so traces don't overlap.
	 * Per-channel trigger-value shifts the baseline vertically.
	 */
	p = devc->dso_buf;
	for (i = devc->dso_sent_samples; i < devc->dso_sent_samples + sending_samples; i++) {
		ch_idx = 0;
		for (l = sdi->channels; l; l = l->next) {
			ch = l->data;
			if (!ch || ch->type != SR_CHANNEL_DSO || !ch->enabled)
				continue;
			enum demo_dso_pattern pat = (ch_idx < DSO_MAX_CHANNELS)
				? devc->dso_pattern[ch_idx] : DEMO_DSO_PATTERN_RANDOM;
			uint8_t v = demo_dso_sample(pat, i,
					mid, amp, ch_idx, wavelength);
			/* Apply per-channel trigger-value offset. */
			if (ch_idx < DSO_MAX_CHANNELS)
				v = (uint8_t)(v + (uint8_t)(devc->dso_trig_value[ch_idx] - 128));
			*p++ = v;
			ch_idx++;
		}
	}

	/* Build and send the DSO packet. */
	dso.data = devc->dso_buf;
	dso.num_samples = sending_samples;
	dso.en_ch_num = en_ch_num;
	dso.sample_bits = devc->dso_unit_bits;
	dso.packet_len = sending_samples * en_ch_num;
	dso.samplerate_tog = (uint32_t)devc->cur_samplerate;

	/* Trigger detection: scan the trigger-source channel's samples for a
	 * crossing of dso_trig_value with the configured slope. If found,
	 * trig_flag=1 and trig_offset marks the crossing sample. If not found,
	 * trig_flag=0 — the GUI will show the waveform without trigger
	 * alignment, causing it to "drift" between frames (auto-trigger).
	 *
	 * The dso_trig_value in the demo is an 8-bit threshold (0-255). The
	 * trigger source is a 0-based DSO channel index. Slope 0=rising,
	 * 1=falling (matching DSO_TRIGGER_RISING/FALLING in dsvdef.h).
	 *
	 * We scan the RAW buffer (before vdiv scaling) because trig_value is
	 * in the same 8-bit space as the generated samples. The vdiv scaling
	 * and coupling applied below are display-only transforms.
	 *
	 * dso_buf layout is interleaved [ch0_s0, ch1_s0, ch0_s1, ...] so
	 * channel ch's sample at index j is at dso_buf[j * en_ch_num + ch].
	 */
	{
		uint8_t trig_ch = devc->dso_trig_source;
		uint8_t trig_level = (trig_ch < DSO_MAX_CHANNELS)
			? devc->dso_trig_value[trig_ch] : 128;
		uint8_t trig_slope = devc->dso_trig_slope; /* 0=rising, 1=falling */
		int16_t trig_pos = -1;

		if (trig_ch < en_ch_num && sending_samples > 1) {
			for (i = 1; i < sending_samples; i++) {
				uint8_t prev = devc->dso_buf[(i - 1) * en_ch_num + trig_ch];
				uint8_t curr = devc->dso_buf[i * en_ch_num + trig_ch];
				if (trig_slope == 0) { /* rising edge */
					if (prev < trig_level && curr >= trig_level) {
						trig_pos = (int16_t)i;
						break;
					}
				} else { /* falling edge */
					if (prev > trig_level && curr <= trig_level) {
						trig_pos = (int16_t)i;
						break;
					}
				}
			}
		}

		if (trig_pos >= 0) {
			dso.trig_flag = 1;
			dso.trig_ch = trig_ch;
			dso.trig_offset = trig_pos;
		} else {
			/* No trigger crossing found: report untriggered. The GUI
			 * will display the waveform at the raw buffer position,
			 * causing it to drift between frames (auto-trigger). */
			dso.trig_flag = 0;
			dso.trig_ch = trig_ch;
			dso.trig_offset = 0;
		}
	}

	/* Apply per-channel vdiv scaling before sending — mirrors old fork
	 * demo's receive_data_dso vdiv transform. The dso_buf layout is
	 * interleaved [ch0_s0, ch1_s0, ch0_s1, ...] so ch_idx = i % 2. */
	demo_dso_vdiv_scale(devc, devc->dso_buf, dso.packet_len);

	/* Apply per-channel coupling (GND/DC/AC) on the final scaled samples.
	 * Without this, switching DSO coupling from DC to GND in the header
	 * had no visible effect — the waveform kept showing the live signal
	 * instead of collapsing to the mid baseline. ANALOG channels already
	 * had this via apply_analog_coupling(); DSO was missing it. */
	apply_dso_coupling(devc, devc->dso_buf, sending_samples, en_ch_num);

	/* Diagnostic: log factor/coupling state every ~50 frames to verify
	 * config changes reach the driver. Downgraded to sr_dbg to avoid
	 * log I/O overhead on the data thread during continuous DSO capture. */
	{
		static int _dso_cfg_dbg = 0;
		if ((++_dso_cfg_dbg % 50) == 0) {
			sr_dbg("[DSO-CFG] vf[0]=%llu vf[1]=%llu coup[0]=%u coup[1]=%u "
			       "en[0]=%d en[1]=%d amp=%u",
			       (unsigned long long)devc->dso_vfactor[0],
			       (unsigned long long)devc->dso_vfactor[1],
			       devc->dso_coupling[0], devc->dso_coupling[1],
			       (int)devc->dso_enabled[0], (int)devc->dso_enabled[1],
			       amp);
		}
	}

	packet.type = SR_DF_DSO;
	packet.payload = &dso;
	sr_session_send(sdi, &packet);

	/* Update DSO measurement stats (max/min/cycle/acc_*) after sending
	 * — mirrors old fork dso_status_update() call in receive_data_dso. */
	demo_dso_status_update(devc, devc->dso_buf, dso.packet_len);

	devc->dso_sent_samples += sending_samples;

	return SR_OK;
}

/* Callback handling data */
SR_PRIV int demo_prepare_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	struct analog_gen *ag;
	GHashTableIter iter;
	void *value;
	uint64_t samples_todo, logic_done, analog_done, analog_sent, sending_now;
	int64_t elapsed_us, limit_us, todo_us;
	int64_t trigger_offset;
	int pre_trigger_samples;

	(void)fd;
	(void)revents;

	sdi = cb_data;
	devc = sdi->priv;

	/* .demo file replay: if a non-random sample source with an open
	 * archive is set, route to the mode-specific receive_data_*_file
	 * function instead of the math-generator streaming path below. */
	if (devc->sample_generator != DEMO_GEN_RANDOM && devc->archive) {
		switch (devc->device_mode) {
		case DEMO_MODE_LOGIC:
			return demo_receive_data_logic_decoder(sdi);
		case DEMO_MODE_DSO:
			return demo_receive_data_dso_file(sdi);
		case DEMO_MODE_ANALOG:
			return demo_receive_data_analog_file(sdi);
		default:
			break;
		}
	}

	static int _demo_tick_count = 0;
	_demo_tick_count++;
	/* Reduced to sr_dbg: with 25ms tick interval this fires 40x/sec,
	 * sr_info would flood the log. */
	sr_dbg("demo_prepare_data TICK #%d: stl=%p, trigger_fired=%d, "
		"limit_samples=%" PRIu64 ", limit_msec=%" PRIu64 ", "
		"sent_samples=%" PRIu64 ", spent_us=%" PRId64 ", "
		"cur_samplerate=%" PRIu64 ", num_logic=%d, num_analog=%d, "
		"num_dso=%zu, enabled_logic=%zu, enabled_analog=%zu",
		_demo_tick_count, (void*)devc->stl, (int)devc->trigger_fired,
		devc->limit_samples, devc->limit_msec,
		devc->sent_samples, devc->spent_us,
		devc->cur_samplerate, devc->num_logic_channels,
		devc->num_analog_channels, (size_t)devc->num_dso_channels,
		devc->enabled_logic_channels, devc->enabled_analog_channels);

	/*
	 * DSO mode: send one complete frame per acquisition tick.
	 * DSO channels coexist with logic/analog in the channel list, but in
	 * DSO work mode only DSO channels are enabled — detect that and skip
	 * the streaming logic/analog path entirely.
	 *
	 * In single-shot mode (non-instant, non-loop): send one full frame then
	 * stop. In instant mode: send progressive slices (live scrolling). In
	 * loop mode: send full frames continuously (timer stays alive).
	 */
	if (devc->num_dso_channels > 0) {
		gboolean has_enabled_dso = FALSE;
		gboolean has_enabled_other = FALSE;
		for (GSList *l = sdi->channels; l; l = l->next) {
			struct sr_channel *ch = l->data;
			if (!ch || !ch->enabled)
				continue;
			if (ch->type == SR_CHANNEL_DSO)
				has_enabled_dso = TRUE;
			else
				has_enabled_other = TRUE;
		}
		if (has_enabled_dso && !has_enabled_other) {
			/* Pure DSO acquisition. */
			demo_send_dso_packet(sdi);
			/* When a full frame has been sent, reset the counter so the
			 * next tick starts a fresh frame (instant mode wraps mid-frame). */
			if (devc->dso_sent_samples >= DSO_PACKET_LEN)
				devc->dso_sent_samples = 0;
			/* DSO 连续采集语义（与 DSView 一致）：
			 *   - instant=TRUE: 单帧滚动采集，发完一帧后停止
			 *     （应用层 Instant Stop 按钮触发）
			 *   - instant=FALSE: 持续采集，每 tick 发送一帧刷新波形，
			 *     不自动停止（应用层 Run/Stop 按钮触发，用户点 Stop 才停）
			 *
			 * 原代码在 !instant && !loop_mode 时单帧停止，导致 DSO 模式
			 * 点 Run/Stop 后只采一帧就停了。DSView 的 demo 驱动在非 instant
			 * 模式下持续发送帧不停止，由应用层 stop_capture 控制停止。
			 * loop_mode 逻辑已包含在"非 instant 持续发送"中，无需单独判断。
			 */
			if (devc->instant) {
				/* Instant: single frame, then stop. */
				sr_dev_acquisition_stop(sdi);
			}
			/* Non-instant: timer stays alive, next tick sends next frame. */
			return G_SOURCE_CONTINUE;
		}
	}

	/* Just in case. */
	if (devc->cur_samplerate <= 0
			|| (devc->num_logic_channels <= 0
			&& devc->num_analog_channels <= 0)) {
		sr_info("demo_prepare_data: EARLY STOP (samplerate=%" PRIu64
			", num_logic=%d, num_analog=%d)",
			devc->cur_samplerate, devc->num_logic_channels,
			devc->num_analog_channels);
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* What time span should we send samples for? */
	elapsed_us = g_get_monotonic_time() - devc->start_us;
	limit_us = 1000 * devc->limit_msec;
	if (limit_us > 0 && limit_us < elapsed_us)
		todo_us = MAX(0, limit_us - devc->spent_us);
	else
		todo_us = MAX(0, elapsed_us - devc->spent_us);

	/* How many samples are outstanding since the last round? */
	samples_todo = (todo_us * devc->cur_samplerate + G_USEC_PER_SEC - 1)
			/ G_USEC_PER_SEC;

	/* Cap samples per tick to keep the callback lightweight.
	 *
	 * At high sample rates (e.g. 1 GHz), a 25 ms tick produces
	 * samples_todo = 25 M — far too many to generate + convert + send
	 * in one tick. The while-loop below would iterate ~12 000 times,
	 * each calling logic_generator + convert_to_cross_data +
	 * sr_session_send, totalling ~400 M iterations for the conversion
	 * alone. This blocks the session worker thread for seconds,
	 * preventing stop-request processing and causing backpressure
	 * stalls that freeze the UI.
	 *
	 * Capping to a fixed maximum keeps each tick lightweight. The
	 * driver falls behind real-time at high rates (the capture takes
	 * longer in wall-clock time), but data flows smoothly, stop is
	 * responsive, and the UI stays interactive. This is the expected
	 * trade-off for a software simulator — real hardware uses DMA.
	 *
	 * 8 M samples/tick: with 1 MB packets (LOGIC_BUFSIZE), a 32-channel
	 * tick produces ~31 sr_session_send calls (down from 122 with 64KB
	 * packets). Each call sends 1 MB — matching pxlogic's large-packet
	 * strategy. At 1 GHz this yields ~320 M samples/s = 320 ms of data
	 * per wall-clock second. */
#define DEMO_MAX_SAMPLES_PER_TICK 8000000
	if (samples_todo > DEMO_MAX_SAMPLES_PER_TICK)
		samples_todo = DEMO_MAX_SAMPLES_PER_TICK;

	/* In loop mode, skip limit_samples clipping — data flows continuously
	 * without stopping at limit_samples, matching pxlogic's is_loop=1
	 * behavior where the samples_counter check is skipped entirely. */
	if (!devc->loop_mode && devc->limit_samples > 0) {
		if (devc->limit_samples < devc->sent_samples)
			samples_todo = 0;
		else if (devc->limit_samples - devc->sent_samples < samples_todo)
			samples_todo = devc->limit_samples - devc->sent_samples;
	}

	sr_dbg("demo_prepare_data: elapsed_us=%" PRId64 ", limit_us=%" PRId64
		", todo_us=%" PRId64 ", samples_todo=%" PRIu64,
		elapsed_us, limit_us, todo_us, samples_todo);

	if (samples_todo == 0)
		return G_SOURCE_CONTINUE;

	if (devc->limit_frames) {
		/* Never send more samples than a frame can fit... */
		samples_todo = MIN(samples_todo, SAMPLES_PER_FRAME);
		/* ...or than we need to finish the current frame. */
		samples_todo = MIN(samples_todo,
			SAMPLES_PER_FRAME - devc->sent_frame_samples);
	}

	/* LA_CROSS_DATA group = 64 采样. samples_todo 必须为 64 的倍数, 否则
	 * fast path 的 cross_samples = (sending_now/64)*64 截断, 而 sent_samples
	 * 推进完整 sending_now → 每 tick 丢失 sending_now % 64 个采样, demo 侧
	 * 采样号 (start_sample_index) 与 PXView 实际收到数持续错位 → 周期性尖峰
	 * (ALL_HIGH 向下 / ALL_LOW 向上). 向下取整到 64 倍数, 无余数丢失.
	 * 副作用: 采集末尾最多早停 63 采样 (limit_samples 判定用 >=, 可接受). */
	samples_todo = (samples_todo / 64) * 64;

	/* Calculate the actual time covered by this run back from the sample
	 * count, rounded towards zero. This avoids getting stuck on a too-low
	 * time delta with no samples being sent due to round-off.
	 *
	 * MUST be computed AFTER the 64-alignment floor above: spent_us may
	 * only be credited for samples that can actually be sent this tick.
	 * At samplerates < 2560 Hz a 25 ms tick yields < 64 samples, so the
	 * floor produces 0 — crediting the unfloored todo_us anyway would
	 * advance spent_us each tick while never sending anything, deadlocking
	 * the capture at 0 samples forever (timed captures never reach
	 * limit_samples → no SR_DF_END). With the credit based on the floored
	 * count, the unsent time accumulates as debt until >= 64 samples are
	 * due, then a burst is sent — low-rate captures complete in bursts. */
	todo_us = samples_todo * G_USEC_PER_SEC / devc->cur_samplerate;

	logic_done = devc->num_logic_channels > 0 ? 0 : samples_todo;
	if (!devc->enabled_logic_channels)
		logic_done = samples_todo;

	analog_done = devc->num_analog_channels > 0 ? 0 : samples_todo;
	if (!devc->enabled_analog_channels)
		analog_done = samples_todo;

	/* Pre-compute fast path eligibility.
	 *
	 * logic_generator_cross() handles ALL pattern types, disabled
	 * channels (fixup), and PWM override directly in LA_CROSS_DATA
	 * format — no intermediate interleaved buffer needed. The only
	 * case that requires the standard path is when a trigger is
	 * active and not yet fired (soft_trigger_logic_check needs
	 * interleaved data in logic_data).
	 *
	 * This eliminates 3 memory passes (generate → fixup → convert)
	 * and replaces them with 1 pass (generate directly in cross). */
	int direct_cross = (!devc->stl || devc->trigger_fired);

	while (logic_done < samples_todo || analog_done < samples_todo) {
		/* Logic */
		if (logic_done < samples_todo) {
			sending_now = MIN(samples_todo - logic_done,
					LOGIC_BUFSIZE / devc->logic_unitsize);

			packet.type = SR_DF_LOGIC;
			packet.payload = &logic;
			logic.unitsize = devc->logic_unitsize;

			if (direct_cross) {
				/* ═══ Fast path: direct LA_CROSS_DATA generation ═══
				 * logic_generator_cross handles all pattern types,
				 * disabled channels (fixup), and PWM override in one
				 * pass — no logic_generator/fixup/convert needed. */
				uint64_t cross_samples = (sending_now / 64) * 64;
				if (cross_samples > 0) {
					uint64_t cross_chunk = (uint64_t)devc->enabled_logic_channels * 8;
					uint64_t cross_len = (cross_samples / 64) * cross_chunk;
					logic_generator_cross(sdi, cross_samples,
						devc->sent_samples + logic_done);

					if (!devc->loop_mode && devc->limit_samples > 0 &&
						devc->sent_samples + logic_done + cross_samples
								>= devc->limit_samples) {
						if (cross_len > cross_chunk + 3) {
							cross_len -= 3;
							/* Keep per-channel u64 alignment (multiple of 8 bytes).
							 * A bare 3-byte cut makes the payload non-8-aligned; the
							 * receiver's cross fallback loop (while(len >= 8)) then
							 * discards len%8 residual bytes without persisting the
							 * continuation, shifting subsequent frames' per-channel
							 * byte alignment and producing a glitch on constant
							 * channels (ch7's flat-high line in the sigrok pattern). */
							cross_len &= ~(uint64_t)7;
						}
					}

					logic.length = cross_len;
					logic.data = devc->cross_data_buf;
					logic.format = LA_CROSS_DATA;
					sr_session_send(sdi, &packet);
				}
				/* 修复: logic_done 推进实际发送的 cross_samples (64 对齐截断),
				 * 而非完整的 sending_now. 否则每 tick 丢失 sending_now % 64 个采样,
				 * 导致下一 payload 的 start_sample_index 与 CROSS group 边界错位
				 * → 接收端每 payload (约 25ms) 出现一次周期性尖峰.
				 * 余数由下一个 tick 的 samples_todo - logic_done 自然补齐.
				 * cross_samples==0 (sending_now<64, 不足一个 CROSS group) 时无法
				 * 发送, 需推进 sending_now 避免死循环 (这 <64 个余数采样被丢弃). */
				logic_done += cross_samples > 0 ? cross_samples : sending_now;
			} else {
			/* ═══ Standard path: generate interleaved + convert ═══ */
				logic_generator(sdi, sending_now * devc->logic_unitsize);

				trigger_offset = 0;
				/* Trigger check: runs in BOTH modes. In Stream mode the trigger
				 * just inserts a SR_DF_TRIGGER marker into the stream (via
				 * std_session_send_df_trigger inside soft_trigger_logic_check).
				 * In Buffer mode it gates data emission until fire. */
				if (devc->stl && (!devc->trigger_fired)) {
				/* Per-tick trigger check log — sr_dbg to avoid flooding at 25ms ticks. */
				sr_dbg("demo trigger check: sending_now=%" PRIu64
					", logic_unitsize=%zu, data[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x"
					", stl->count=%d, op_mode=%d",
					sending_now, devc->logic_unitsize,
					devc->logic_data[0], devc->logic_data[1],
					devc->logic_data[2], devc->logic_data[3],
					devc->logic_data[4], devc->logic_data[5],
					devc->logic_data[6], devc->logic_data[7],
					devc->stl->count, devc->op_mode);
					trigger_offset = soft_trigger_logic_check(devc->stl,
							devc->logic_data, sending_now * devc->logic_unitsize,
							&pre_trigger_samples);
				sr_dbg("demo trigger check: soft_trigger_logic_check returned %d"
					", pre_trigger_samples=%d",
					(int)trigger_offset, pre_trigger_samples);
					if (trigger_offset > -1) {
						devc->trigger_fired = TRUE;
						sr_info("demo trigger FIRED at offset %d (op_mode=%d)",
							(int)trigger_offset, devc->op_mode);
						/* In Buffer mode, reset logic_done to pre_trigger_samples
						 * so pre-trigger data (already sent by soft_trigger_logic_check)
						 * isn't double-counted. In Stream mode we already sent the
						 * full buffer above, so just mark fired and continue. */
						if (devc->op_mode == DEMO_OP_BUFFER)
							logic_done = pre_trigger_samples;
					}
				}

				/* Determine data range based on trigger state */
				uint8_t *logic_src = devc->logic_data;
				uint64_t logic_send_samples = sending_now;

				if (devc->stl && devc->op_mode == DEMO_OP_BUFFER) {
					/* Buffer mode: only send AFTER trigger fires. */
					if (devc->trigger_fired && (trigger_offset < (int)sending_now)) {
						logic_src = devc->logic_data + trigger_offset * devc->logic_unitsize;
						logic_send_samples = sending_now - trigger_offset;
					} else {
						logic_send_samples = 0;
						/* Trigger not yet fired: send nothing but advance
						 * logic_done so we don't loop forever. */
						logic_done += sending_now;
					}
				}

				if (logic_send_samples > 0) {
					/* Apply fixup on sample-interleaved data (masks disabled
					 * channel bits in-place). */
					logic.length = logic_send_samples * devc->logic_unitsize;
					logic.data = logic_src;
					logic_fixup_feed(devc, &logic);

					/* Convert to LA_CROSS_DATA channel-block format.
					 * Round down to a multiple of 64 (one chunk = 64 samples
					 * per channel). Any remainder is deferred to next tick. */
					uint64_t cross_samples = (logic_send_samples / 64) * 64;
					if (cross_samples > 0) {
						uint64_t cross_chunk = (uint64_t)devc->enabled_logic_channels * 8;
						uint64_t cross_len = (cross_samples / 64) * cross_chunk;
						convert_to_cross_data(logic_src, devc->cross_data_buf,
							cross_samples, devc->logic_unitsize,
							devc->enabled_logic_channels);

						/* Simulate real hardware: the last packet before stop
						 * often ends with a partial chunk (not a multiple of
						 * channel_num * 8 bytes), because USB transfers are
						 * sized by the DMA engine. This leaves _ch_fraction
						 * or _byte_fraction non-zero, triggering the bit-align
						 * phase in append_cross_payload on the next packet. */
						if (!devc->loop_mode && devc->limit_samples > 0 &&
							devc->sent_samples + logic_done + cross_samples
									>= devc->limit_samples) {
							if (cross_len > cross_chunk + 3) {
								cross_len -= 3;
								/* Keep per-channel u64 alignment (multiple of 8 bytes).
								 * A bare 3-byte cut makes the payload non-8-aligned; the
								 * receiver's cross fallback loop (while(len >= 8)) then
								 * discards len%8 residual bytes without persisting the
								 * continuation, shifting subsequent frames' per-channel
								 * byte alignment and producing a glitch on constant
								 * channels (ch7's flat-high line in the sigrok pattern). */
								cross_len &= ~(uint64_t)7;
							}
						}

						logic.length = cross_len;
						logic.data = devc->cross_data_buf;
						logic.format = LA_CROSS_DATA;
						sr_session_send(sdi, &packet);
					}
					/* 修复: 与 fast path 一致, 推进实际发送的 cross_samples 而非
					 * logic_send_samples, 避免每 payload 余数丢失导致 CROSS group
					 * 边界错位 (周期尖峰). cross_samples==0 时推进完整避免死循环. */
					logic_done += cross_samples > 0 ? cross_samples : logic_send_samples;
				}
			} /* end standard path */
		}

		/* Analog, one channel at a time */
		if (analog_done < samples_todo) {
			analog_sent = 0;

			g_hash_table_iter_init(&iter, devc->ch_ag);
			while (g_hash_table_iter_next(&iter, NULL, &value)) {
				send_analog_packet(value, sdi, &analog_sent,
						devc->sent_samples + analog_done,
						samples_todo - analog_done);
			}
			analog_done += analog_sent;
		}
	}

	uint64_t min = MIN(logic_done, analog_done);
	/* sent_samples accumulation policy:
	 * - No trigger (stl==NULL): always accumulate (both modes).
	 * - Stream mode: always accumulate — data is being sent continuously,
	 *   trigger marker or not. limit_samples is the stop condition.
	 * - Buffer mode + trigger not fired: do NOT accumulate — keeps the
	 *   session alive waiting for trigger. spent_us still accumulates so
	 *   limit_msec acts as a timeout. */
	if (!devc->stl || devc->op_mode == DEMO_OP_STREAM || devc->trigger_fired) {
		devc->sent_samples += min;
	}
	devc->sent_frame_samples += min;
	devc->spent_us += todo_us;

	if (devc->limit_frames && devc->sent_frame_samples >= SAMPLES_PER_FRAME) {
		std_session_send_df_frame_end(sdi);
		devc->sent_frame_samples = 0;
		devc->limit_frames--;
		if (!devc->limit_frames) {
			sr_dbg("Requested number of frames reached.");
			sr_dev_acquisition_stop(sdi);
		}
	}

	/* Stop condition: limit_samples only applies when NOT in loop mode
	 * (matching pxlogic's is_loop=1 which skips the samples_counter check
	 * entirely — data flows forever until user stops). limit_msec always
	 * applies as a hard timeout, even in loop mode. */
	if ((!devc->loop_mode && devc->limit_samples > 0 && devc->sent_samples >= devc->limit_samples)
			|| (limit_us > 0 && devc->spent_us >= limit_us)) {

		/* If we're averaging everything - now is the time to send data */
		if (devc->avg && devc->avg_samples == 0) {
			g_hash_table_iter_init(&iter, devc->ch_ag);
			while (g_hash_table_iter_next(&iter, NULL, &value)) {
				ag = value;
				packet.type = SR_DF_ANALOG;
				packet.payload = &ag->packet;
				/* 应用耦合 (与 do_send 路径一致)。 */
				apply_analog_coupling(ag, devc, &ag->avg_val, 1);
				ag->packet.data = &ag->avg_val;
				ag->packet.num_samples = 1;
				sr_session_send(sdi, &packet);
			}
		}
		sr_info("demo_prepare_data: STOP condition met (sent_samples=%" PRIu64
			", limit_samples=%" PRIu64 ", spent_us=%" PRId64
			", limit_us=%" PRId64 ", loop_mode=%d)",
			devc->sent_samples, devc->limit_samples,
			devc->spent_us, limit_us, (int)devc->loop_mode);
		sr_dev_acquisition_stop(sdi);
	} else if (devc->limit_frames) {
		if (devc->sent_frame_samples == 0)
			std_session_send_df_frame_begin(sdi);
	}

	return G_SOURCE_CONTINUE;
}

/* =====================================================================
 * .demo file replay support (ported from old fork demo driver).
 * The .demo file format is a zip archive containing:
 *   - 'header'        : INI metadata (samplerate / total samples / total
 *                       blocks / total probes / probeN names)
 *   - L-<ch>/<block> : per-channel per-block logic data (byte-interleaved)
 *   - O-<ch>/0       : per-channel single-block DSO data
 *   - A-0/0          : single-channel single-block analog data
 * Files live under <user_config_dir>/demo/<mode>/<name>.demo.
 * ===================================================================== */

/* Return the demo file root directory: <user_config_dir>/demo/.
 * Caller owns the returned string (g_free). Mirrors old fork's
 * DS_USR_PATH/demo/ layout. */
SR_PRIV char *demo_get_root_dir(void)
{
	const gchar *cfg_dir;
	char *path;

	cfg_dir = g_get_user_config_dir();
	if (!cfg_dir)
		cfg_dir = ".";
	path = g_build_filename(cfg_dir, "demo", NULL);
	return path;
}

/* Close any open zip archive. Safe to call when archive is NULL. */
SR_PRIV void demo_close_archive(struct dev_context *devc)
{
	if (!devc)
		return;
	if (devc->archive) {
		unzClose(devc->archive);
		devc->archive = NULL;
	}
}

/* Scan <root>/<sub_dir>/ for *.demo files and store their basenames
 * (without the .demo suffix) into info->patterns[]. Index 0 is always
 * the static literal "random" (DEMO_GEN_RANDOM). Returns SR_OK even when
 * the directory does not exist (only "random" available in that case). */
SR_PRIV int demo_get_pattern_mode_from_file(const char *sub_dir,
	struct demo_mode_pattern *info, int max_count)
{
	char *root, *dir_path, *full, *name;
	GDir *dir;
	const char *filename;
	int num = 1;

	if (!sub_dir || !info || max_count < 1)
		return SR_ERR_ARG;

	/* Index 0 is always the literal "random" — never freed by clear_helper
	 * (which only frees indices >= 1). */
	info->patterns[0] = (char *)"random";
	info->count = 1;

	root = demo_get_root_dir();
	dir_path = g_build_filename(root, sub_dir, NULL);
	g_free(root);

	dir = g_dir_open(dir_path, 0, NULL);
	if (!dir) {
		g_free(dir_path);
		return SR_OK;  /* directory missing — only random available */
	}

	while ((filename = g_dir_read_name(dir)) != NULL && num < max_count) {
		full = g_build_filename(dir_path, filename, NULL);
		if (!g_file_test(full, G_FILE_TEST_IS_DIR) &&
		    g_str_has_suffix(filename, ".demo")) {
			/* Strip the ".demo" suffix (5 chars). */
			name = g_strndup(filename, strlen(filename) - 5);
			info->patterns[num] = name;
			num++;
		}
		g_free(full);
	}
	g_dir_close(dir);
	g_free(dir_path);
	info->count = num;
	return SR_OK;
}

/* Find the index of str in devc->demo_pattern_array[mode].patterns[].
 * Returns the matching index, or -1 if not found. */
SR_PRIV int demo_get_pattern_mode_index_by_string(struct dev_context *devc,
	int mode, const char *str)
{
	int i;
	struct demo_mode_pattern *info;

	if (!devc || !str || mode < 0 || mode > 2)
		return -1;

	info = &devc->demo_pattern_array[mode];
	for (i = 0; i < info->count; i++) {
		if (info->patterns[i] && strcmp(info->patterns[i], str) == 0)
			return i;
	}
	return -1;
}

/* Build the absolute path to the .demo file for the given pattern_mode and
 * store it in devc->demo_file_path (g_free'd before re-alloc). Returns
 * SR_ERR if pattern_mode is out of range; SR_OK otherwise. */
SR_PRIV int demo_reset_dsl_path(struct sr_dev_inst *sdi, uint8_t pattern_mode)
{
	struct dev_context *devc = sdi->priv;
	static const char *const mode_names[] = { "logic", "dso", "analog" };
	struct demo_mode_pattern *info;
	char *root, *mode_dir, *file_path = NULL;
	int mode = (int)devc->device_mode;

	g_free(devc->demo_file_path);

	if (pattern_mode != DEMO_GEN_RANDOM) {
		if (mode < 0 || mode > 2) {
			devc->demo_file_path = g_strdup("");
			return SR_ERR;
		}
		info = &devc->demo_pattern_array[mode];
		if (pattern_mode >= (uint8_t)info->count || !info->patterns[pattern_mode]) {
			devc->demo_file_path = g_strdup("");
			return SR_ERR;
		}
		root = demo_get_root_dir();
		mode_dir = g_build_filename(root, mode_names[mode], NULL);
		file_path = g_strdup_printf("%s/%s.demo",
			mode_dir, info->patterns[pattern_mode]);
		g_free(mode_dir);
		g_free(root);
		devc->demo_file_path = file_path;
	} else {
		/* Math-generated mode — no file. */
		devc->demo_file_path = g_strdup("");
	}
	return SR_OK;
}

/* First-open initialization: scan all three mode subdirectories for .demo
 * files, then pick the default LOGIC pattern (DEFAULT_LOGIC_FILE). */
SR_PRIV void demo_scan_dsl_file(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	int dex;

	if (!devc->b_load_directory) {
		demo_get_pattern_mode_from_file("logic",
			&devc->demo_pattern_array[DEMO_MODE_LOGIC], PATTERN_COUNT);
		demo_get_pattern_mode_from_file("dso",
			&devc->demo_pattern_array[DEMO_MODE_DSO], PATTERN_COUNT);
		demo_get_pattern_mode_from_file("analog",
			&devc->demo_pattern_array[DEMO_MODE_ANALOG], PATTERN_COUNT);
		devc->b_load_directory = TRUE;
	}

	dex = demo_get_pattern_mode_index_by_string(devc, DEMO_MODE_LOGIC,
		DEFAULT_LOGIC_FILE);
	if (dex == -1)
		dex = DEMO_GEN_RANDOM;
	devc->sample_generator = (uint8_t)dex;
	devc->device_mode = DEMO_MODE_LOGIC;
	demo_reset_dsl_path(sdi, devc->sample_generator);
}

/* Open the .demo zip archive and parse its 'header' INI to populate
 * devc->cur_samplerate / total_samples / limit_samples / num_blocks /
 * num_probes. Channels are NOT rebuilt (the new design toggles ch->enabled
 * instead — see api.c config_set SR_CONF_DEVICE_MODE). If the archive
 * cannot be opened or has no header, falls back to RANDOM mode with
 * mode-specific defaults. Sets devc->load_data = TRUE on success so the
 * receive_data_*_file functions know data is ready to be read. */
SR_PRIV int demo_load_virtual_device_session(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	char *metafile = NULL;
	GKeyFile *kf = NULL;
	char **sections = NULL, **keys = NULL, *val = NULL;
	gsize ngroups, nkeys;
	gsize i, j;

	demo_close_archive(devc);
	devc->cur_block = 0;

	if (devc->sample_generator != DEMO_GEN_RANDOM &&
	    devc->demo_file_path && devc->demo_file_path[0] != '\0') {
		/* Open the .demo zip archive. */
		devc->archive = unzOpen64(devc->demo_file_path);
		if (!devc->archive) {
			sr_warn("demo: failed to open '%s', falling back to random.",
				devc->demo_file_path);
			devc->sample_generator = DEMO_GEN_RANDOM;
			goto set_defaults;
		}

		/* Locate the 'header' file inside the zip. */
		if (unzLocateFile(devc->archive, "header", 0) != UNZ_OK) {
			sr_warn("demo: no 'header' in '%s', falling back to random.",
				devc->demo_file_path);
			demo_close_archive(devc);
			devc->sample_generator = DEMO_GEN_RANDOM;
			goto set_defaults;
		} else {
			unz_file_info64 info;
			if (unzGetCurrentFileInfo64(devc->archive, &info, NULL, 0,
					NULL, 0, NULL, 0) != UNZ_OK) {
				sr_warn("demo: unzGetCurrentFileInfo64 failed, "
					"falling back to random.");
				demo_close_archive(devc);
				devc->sample_generator = DEMO_GEN_RANDOM;
				goto set_defaults;
			}
			metafile = g_malloc(info.uncompressed_size + 1);
			if (!metafile) {
				demo_close_archive(devc);
				devc->sample_generator = DEMO_GEN_RANDOM;
				goto set_defaults;
			}
			unzOpenCurrentFile(devc->archive);
			unzReadCurrentFile(devc->archive, metafile,
				info.uncompressed_size);
			metafile[info.uncompressed_size] = '\0';
			unzCloseCurrentFile(devc->archive);
		}

		/* Parse the header INI. probeN keys are skipped — the new
		 * design does not rebuild channels on file load. */
		kf = g_key_file_new();
		if (g_key_file_load_from_data(kf, metafile, strlen(metafile),
				G_KEY_FILE_NONE, NULL)) {
			sections = g_key_file_get_groups(kf, &ngroups);
			for (i = 0; i < ngroups; i++) {
				keys = g_key_file_get_keys(kf, sections[i], &nkeys, NULL);
				for (j = 0; j < nkeys; j++) {
					val = g_key_file_get_value(kf, sections[i],
						keys[j], NULL);
					if (!val)
						continue;
					if (!strcmp(keys[j], "samplerate")) {
						devc->cur_samplerate =
							g_ascii_strtoull(val, NULL, 10);
					} else if (!strcmp(keys[j], "total samples")) {
						devc->total_samples =
							g_ascii_strtoull(val, NULL, 10);
						devc->limit_samples = devc->total_samples;
					} else if (!strcmp(keys[j], "total blocks")) {
						devc->num_blocks = (int)g_ascii_strtoull(
							val, NULL, 10);
					} else if (!strcmp(keys[j], "total probes")) {
						devc->num_probes = (int)g_ascii_strtoull(
							val, NULL, 10);
					}
					/* probe0/probe1/... channel-name keys
					 * intentionally ignored. */
					g_free(val);
					val = NULL;
				}
				g_strfreev(keys);
				keys = NULL;
			}
			g_strfreev(sections);
			sections = NULL;
		} else {
			sr_warn("demo: failed to parse header INI, "
				"using parsed values where available.");
		}
		g_key_file_free(kf);
		g_free(metafile);
	}

set_defaults:
	/* RANDOM fallback (or explicit random mode): set mode-specific
	 * defaults so the math generators have a sensible capture depth. */
	if (devc->sample_generator == DEMO_GEN_RANDOM) {
		switch (devc->device_mode) {
		case DEMO_MODE_LOGIC:
			devc->cur_samplerate = SR_MHZ(1);
			devc->total_samples = SR_MHZ(1);
			devc->limit_samples = devc->total_samples;
			devc->num_blocks = 0;
			break;
		case DEMO_MODE_DSO:
			devc->cur_samplerate = SR_MHZ(100);
			devc->total_samples = SR_KHZ(10);
			devc->limit_samples = devc->total_samples;
			devc->num_blocks = 1;
			break;
		case DEMO_MODE_ANALOG:
			devc->cur_samplerate = SR_KHZ(200);
			devc->total_samples = SR_KHZ(10);
			devc->limit_samples = devc->total_samples;
			devc->num_blocks = 1;
			break;
		default:
			break;
		}
	}
	devc->load_data = TRUE;
	return SR_OK;
}

/* DSO measurement calculation (ported from old fork dso_status_update).
 * Walks the interleaved [ch0,ch1,...] DSO buffer and computes per-channel
 * max/min/high_level/low_level/cycle stats/accumulated mean/square into
 * devc->mstatus. ch0 and ch1 share the same values (the old fork's
 * behavior). Called after each SR_DF_DSO packet by demo_send_dso_packet
 * and demo_receive_data_dso_file. */
SR_PRIV void demo_dso_status_update(struct dev_context *devc,
	const uint8_t *buf, uint32_t len)
{
	struct demo_dso_measure *st;
	uint8_t ch_max = DSO_MID_VAL;
	uint8_t ch_min = DSO_MID_VAL;
	uint8_t val;
	uint32_t i;
	uint64_t total_val = 0;
	gboolean first_plevel = FALSE;
	gboolean temp_plevel = FALSE;
	uint32_t temp_llen = 0;

	if (!devc || !buf || len == 0)
		return;
	st = &devc->mstatus;

	/* Step 1: find max/min over ch0 samples (every other byte). */
	for (i = 0; i < len; i += 2) {
		val = buf[i];
		if (val > ch_max)
			ch_max = val;
		if (val < ch_min)
			ch_min = val;
	}
	st->ch0_max = st->ch1_max = ch_max;
	st->ch0_min = st->ch1_min = ch_min;
	st->ch0_high_level = st->ch1_high_level = ch_max;
	st->ch0_low_level = st->ch1_low_level = ch_min;

	/* Step 2: clear cycle stats. */
	st->ch0_cyc_tlen = st->ch1_cyc_tlen = 0;
	st->ch0_cyc_llen = st->ch1_cyc_llen = 0;
	st->ch0_cyc_cnt = st->ch1_cyc_cnt = 0;
	st->ch0_cyc_rlen = st->ch1_cyc_rlen = 0;
	st->ch0_cyc_flen = st->ch1_cyc_flen = 0;
	st->ch0_cyc_plen = st->ch1_cyc_plen = 0;
	st->ch0_level_valid = st->ch1_level_valid = TRUE;
	st->ch0_acc_mean = st->ch1_acc_mean = 0;

	/* Step 3: scan for cycle stats. ch0 and ch1 share the same values
	 * (mirrors old fork behavior where both channels are updated in lockstep). */
	for (i = 0; i < len; i += 2) {
		val = buf[i];
		if (first_plevel) {
			if (val <= DSO_MID_VAL) {
				st->ch0_cyc_plen++;
				st->ch1_cyc_plen++;
			}
			temp_llen++;
			if (temp_plevel) {
				/* currently high — count rising edge */
				st->ch0_cyc_rlen++;
				st->ch1_cyc_rlen++;
				if (val == ch_max) {
					temp_plevel = !temp_plevel;
					if (st->ch0_plevel == temp_plevel) {
						st->ch0_cyc_cnt++;
						st->ch1_cyc_cnt++;
						st->ch0_cyc_tlen += temp_llen;
						st->ch1_cyc_tlen += temp_llen;
						st->ch0_cyc_llen = st->ch1_cyc_llen = 0;
						temp_llen = 0;
					} else {
						st->ch0_cyc_llen = st->ch1_cyc_llen = temp_llen;
					}
				}
			} else {
				/* currently low — count falling edge */
				st->ch0_cyc_flen++;
				st->ch1_cyc_flen++;
				if (val == ch_min) {
					temp_plevel = !temp_plevel;
					if (st->ch0_plevel == temp_plevel) {
						st->ch0_cyc_cnt++;
						st->ch1_cyc_cnt++;
						st->ch0_cyc_tlen += temp_llen;
						st->ch1_cyc_tlen += temp_llen;
						st->ch0_cyc_llen = st->ch1_cyc_llen = 0;
						temp_llen = 0;
					} else {
						st->ch0_cyc_llen = st->ch1_cyc_llen = temp_llen;
					}
				}
			}
		} else {
			/* Find first extremum to seed plevel. */
			if (val == ch_max || val == ch_min) {
				if (val == ch_max) {
					temp_plevel = FALSE;
					st->ch0_plevel = st->ch1_plevel = FALSE;
				} else {
					temp_plevel = TRUE;
					st->ch0_plevel = st->ch1_plevel = TRUE;
				}
				first_plevel = TRUE;
			}
		}
		total_val += val;
	}

	/* Step 4: rlen/flen halve (each cycle counted twice: rising+falling). */
	st->ch0_cyc_rlen /= 2;
	st->ch1_cyc_rlen = st->ch0_cyc_rlen;
	st->ch0_cyc_flen /= 2;
	st->ch1_cyc_flen = st->ch0_cyc_flen;

	/* Step 5: acc_mean / acc_square. For non-RANDOM mode the waveform is
	 * a uniform mid value, so acc_mean = mid * num_ch0_samples. For
	 * RANDOM mode, acc_mean = sum of all ch0 samples. acc_square is a
	 * rough estimate (ch_max * num_samples * 7) matching old fork. */
	if (devc->sample_generator != DEMO_GEN_RANDOM) {
		st->ch0_acc_mean = st->ch1_acc_mean =
			(uint32_t)(DSO_MID_VAL * (uint64_t)(len / 2));
	} else {
		st->ch0_acc_mean = st->ch1_acc_mean = (uint32_t)total_val;
	}
	st->ch0_acc_square = st->ch1_acc_square =
		(uint64_t)ch_max * (uint64_t)(len / 2) * 7;
	st->measure_valid = TRUE;
}

/* DSO per-channel vdiv scaling (ported from old fork receive_data_dso).
 * Walks the interleaved [ch0,ch1,...] DSO buffer and applies per-channel
 * linear compression (vdiv > 200mV) or expansion-with-clamp (vdiv <= 200mV)
 * using devc->dso_vdiv[] and devc->dso_offset[]. Called by
 * demo_send_dso_packet before sending the SR_DF_DSO packet, and by
 * demo_receive_data_dso_file after loading file data. */
SR_PRIV void demo_dso_vdiv_scale(struct dev_context *devc,
	uint8_t *buf, uint32_t len)
{
	uint32_t i;
	int ch_idx;

	if (!devc || !buf || len == 0)
		return;

	for (i = 0; i < len; i++) {
		uint64_t vdiv;
		uint8_t temp_val = buf[i];
		uint16_t val, tem;

		ch_idx = (int)(i % 2);  /* 0=ch0, 1=ch1 */
		vdiv = devc->dso_vdiv[ch_idx];

		if (vdiv > SR_mV(200)) {
			/* Normal range: linear compress around mid. */
			if (temp_val > DSO_MID_VAL) {
				val = (uint8_t)(temp_val - DSO_MID_VAL);
				tem = (uint16_t)((uint16_t)val *
					(uint16_t)DSO_DEFAULT_VDIV / (uint16_t)vdiv);
				temp_val = (uint8_t)(DSO_MID_VAL + tem);
			} else if (temp_val < DSO_MID_VAL) {
				val = (uint8_t)(DSO_MID_VAL - temp_val);
				tem = (uint16_t)((uint16_t)val *
					(uint16_t)DSO_DEFAULT_VDIV / (uint16_t)vdiv);
				temp_val = (uint8_t)(DSO_MID_VAL - tem);
			}
			buf[i] = temp_val;
		} else {
			/* High sensitivity range: expand to 16-bit then clamp
			 * into the [DSO_MAX_VAL, DSO_MIN_VAL] window around
			 * (expand_mid - offset). */
			uint16_t high_gate, low_gate;
			uint16_t expand_mid;
			uint16_t expand_scale;
			/* (SR_mV(200) / vdiv) * 256 — old fork's
			 * DSO_EXPAND_MID_VAL() macro expands the 8-bit value into
			 * the 16-bit space centered on this point. */
			expand_scale = (uint16_t)(SR_mV(200) / vdiv);
			expand_mid = (uint16_t)(expand_scale * 256);
			if (temp_val > DSO_MID_VAL) {
				val = (uint8_t)(temp_val - DSO_MID_VAL);
				tem = (uint16_t)((uint16_t)val *
					(uint16_t)DSO_DEFAULT_VDIV / (uint16_t)vdiv);
				tem = (uint16_t)(expand_mid + tem);
			} else if (temp_val < DSO_MID_VAL) {
				val = (uint8_t)(DSO_MID_VAL - temp_val);
				tem = (uint16_t)((uint16_t)val *
					(uint16_t)DSO_DEFAULT_VDIV / (uint16_t)vdiv);
				tem = (uint16_t)(expand_mid - tem);
			} else {
				tem = expand_mid;
			}
			high_gate = (uint16_t)(expand_mid - devc->dso_offset[ch_idx]);
			low_gate = (uint16_t)(high_gate + DSO_LIMIT);
			if (tem <= high_gate)
				tem = DSO_MAX_VAL;
			else if (tem >= low_gate)
				tem = DSO_MIN_VAL;
			else
				tem = (uint16_t)(tem - high_gate);
			buf[i] = (uint8_t)tem;
		}
	}
}

/* ANALOG per-channel vdiv scaling (ported from old fork receive_data_analog).
 * Applies linear compression around ANALOG_MID_VAL using
 * devc->analog_vdiv[ch_idx]. Called by send_analog_packet (math mode) and
 * demo_receive_data_analog_file (file mode) before sending SR_DF_ANALOG. */
SR_PRIV void demo_analog_vdiv_scale(struct dev_context *devc,
	uint8_t *buf, uint32_t len, int ch_idx)
{
	uint32_t i;
	uint64_t vdiv;

	if (!devc || !buf || len == 0 || ch_idx < 0 || ch_idx >= DSO_MAX_CHANNELS)
		return;

	vdiv = devc->analog_vdiv[ch_idx];
	if (vdiv == 0)
		return;

	for (i = 0; i < len; i++) {
		uint8_t temp_value = buf[i];
		uint8_t val;
		uint16_t tem;

		if (temp_value > ANALOG_MID_VAL) {
			val = (uint8_t)(temp_value - ANALOG_MID_VAL);
			tem = (uint16_t)((uint16_t)val *
				(uint16_t)ANALOG_DEFAULT_VDIV / (uint16_t)vdiv);
			if (tem >= ANALOG_MID_VAL)
				temp_value = ANALOG_MIN_VAL;
			else
				temp_value = (uint8_t)(ANALOG_MID_VAL + tem);
		} else if (temp_value < ANALOG_MID_VAL) {
			val = (uint8_t)(ANALOG_MID_VAL - temp_value);
			tem = (uint16_t)((uint16_t)val *
				(uint16_t)ANALOG_DEFAULT_VDIV / (uint16_t)vdiv);
			if (tem >= ANALOG_MID_VAL)
				temp_value = ANALOG_MAX_VAL;
			else
				temp_value = (uint8_t)(ANALOG_MID_VAL - tem);
		}
		buf[i] = temp_value;
	}
}

/* =====================================================================
 * .demo zip data replay callbacks.
 * Each function reads per-channel block data from the open zip archive
 * (devc->archive), byte-interleaves it into devc->packet_buffer->post_buf,
 * and sends a SR_DF_LOGIC / SR_DF_DSO / SR_DF_ANALOG packet. Called from
 * demo_prepare_data when sample_generator != DEMO_GEN_RANDOM && archive.
 * Returns G_SOURCE_CONTINUE to keep the timer alive, or stops the
 * acquisition (sr_dev_acquisition_stop) when all blocks have been read.
 * ===================================================================== */

/* Count enabled channels of the given type and fill ch_list[] with pointers
 * to them in sdi->channels order. Returns the count (0..max). */
static int demo_collect_enabled_channels(const struct sr_dev_inst *sdi,
	int channel_type, struct sr_channel **ch_list, int max)
{
	struct sr_channel *ch;
	GSList *l;
	int n = 0;

	for (l = sdi->channels; l && n < max; l = l->next) {
		ch = l->data;
		if (ch && ch->type == channel_type && ch->enabled)
			ch_list[n++] = ch;
	}
	return n;
}

/* Ensure devc->packet_buffer exists and its post_buf is sized for
 * post_buf_len bytes. Allocates / reallocates as needed. Returns NULL on
 * allocation failure. */
static struct demo_packet_buffer *demo_ensure_packet_buffer(
	struct dev_context *devc, uint64_t post_buf_len)
{
	struct demo_packet_buffer *pb;
	int i;

	if (!devc->packet_buffer) {
		devc->packet_buffer = g_new0(struct demo_packet_buffer, 1);
		if (!devc->packet_buffer)
			return NULL;
	}
	pb = devc->packet_buffer;
	if (pb->post_buf_len != post_buf_len) {
		g_free(pb->post_buf);
		pb->post_buf = g_malloc(post_buf_len);
		if (!pb->post_buf) {
			pb->post_buf_len = 0;
			return NULL;
		}
		pb->post_buf_len = post_buf_len;
		pb->post_len = 0;
		for (i = 0; i < MAX_PROBE_NUM; i++) {
			g_free(pb->block_bufs[i]);
			pb->block_bufs[i] = NULL;
			pb->block_read_positions[i] = 0;
		}
		pb->block_data_len = 0;
		pb->block_chan_read_pos = 0;
	}
	return pb;
}

/* Read one per-channel block (file "L-<ch>/<block>") from the zip into
 * pb->block_bufs[ch]. Allocates block_bufs as needed. Returns SR_OK or
 * SR_ERR on zip failure. */
static int demo_read_logic_block(struct dev_context *devc,
	struct demo_packet_buffer *pb, int chan_num, int block_idx)
{
	char file_name[32];
	unz_file_info64 info;
	int ch_index, malloc_idx;
	int ret;

	for (ch_index = 0; ch_index < chan_num; ch_index++) {
		snprintf(file_name, sizeof(file_name), "L-%d/%d",
			ch_index, block_idx);
		if (unzLocateFile(devc->archive, file_name, 0) != UNZ_OK) {
			sr_err("demo: can't locate zip entry '%s'.", file_name);
			return SR_ERR;
		}
		if (unzGetCurrentFileInfo64(devc->archive, &info,
				NULL, 0, NULL, 0, NULL, 0) != UNZ_OK) {
			sr_err("demo: unzGetCurrentFileInfo64 failed for '%s'.",
				file_name);
			return SR_ERR;
		}
		if (ch_index == 0) {
			/* First channel sets the block size; allocate all
			 * channel block_bufs to that size. */
			if (info.uncompressed_size > pb->block_data_len) {
				for (malloc_idx = 0; malloc_idx < chan_num; malloc_idx++) {
					g_free(pb->block_bufs[malloc_idx]);
					pb->block_bufs[malloc_idx] =
						g_malloc(info.uncompressed_size + 1);
					if (!pb->block_bufs[malloc_idx])
						return SR_ERR_MALLOC;
				}
				pb->block_data_len = info.uncompressed_size;
			}
		} else if (info.uncompressed_size != pb->block_data_len) {
			sr_err("demo: block size mismatch for '%s'.", file_name);
			return SR_ERR;
		}
		if (unzOpenCurrentFile(devc->archive) != UNZ_OK) {
			sr_err("demo: can't open zip entry '%s'.", file_name);
			return SR_ERR;
		}
		ret = unzReadCurrentFile(devc->archive,
			pb->block_bufs[ch_index], pb->block_data_len);
		unzCloseCurrentFile(devc->archive);
		if (ret < 0) {
			sr_err("demo: read error for '%s'.", file_name);
			return SR_ERR;
		}
		pb->block_read_positions[ch_index] = 0;
	}
	pb->block_chan_read_pos = 0;
	return SR_OK;
}

/* LOGIC file replay: reads L-<ch>/<block> from zip, byte-interleaves 8
 * bytes per channel per unit into post_buf (LA_CROSS_DATA format), sends
 * SR_DF_LOGIC. Stops acquisition when all blocks are consumed. */
SR_PRIV int demo_receive_data_logic_decoder(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic;
	struct sr_channel *ch_list[MAX_PROBE_NUM];
	struct demo_packet_buffer *pb;
	int chan_num;
	const uint8_t byte_align = 8;  /* LOGIC: 8 bytes per channel per unit */
	uint64_t post_buf_len;
	int read_chan_idx, ret;

	chan_num = demo_collect_enabled_channels(sdi, SR_CHANNEL_LOGIC,
		ch_list, MAX_PROBE_NUM);
	if (chan_num < 1) {
		sr_err("demo: no enabled logic channels for file replay.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* post_buf holds byte_align bytes per channel per interleave unit.
	 * Use a chunk of ~4KB worth of units (512 units per channel). */
	post_buf_len = (uint64_t)byte_align * chan_num * 64;
	pb = demo_ensure_packet_buffer(devc, post_buf_len);
	if (!pb) {
		sr_err("demo: packet buffer alloc failed.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* If the current block has been fully interleaved, advance to the
	 * next block (or stop if all blocks are done). */
	if (pb->block_chan_read_pos >= pb->block_data_len) {
		if (devc->cur_block >= devc->num_blocks) {
			/* All blocks read — end of stream. */
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		ret = demo_read_logic_block(devc, pb, chan_num, devc->cur_block);
		if (ret != SR_OK) {
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		devc->cur_block++;
	}

	/* Byte-interleave one chunk: byte_align bytes from each channel in
	 * turn, until post_buf is full or the block is exhausted. */
	pb->post_len = 0;
	read_chan_idx = 0;
	while (pb->post_len + byte_align <= pb->post_buf_len &&
	       pb->block_chan_read_pos + byte_align <= pb->block_data_len) {
		uint8_t *dst = (uint8_t *)pb->post_buf + pb->post_len;
		uint8_t *src = (uint8_t *)pb->block_bufs[read_chan_idx] +
			pb->block_read_positions[read_chan_idx];
		memcpy(dst, src, byte_align);
		pb->post_len += byte_align;
		pb->block_read_positions[read_chan_idx] += byte_align;
		read_chan_idx++;
		if (read_chan_idx == chan_num) {
			read_chan_idx = 0;
			pb->block_chan_read_pos += byte_align;
		}
	}

	if (pb->post_len >= (uint64_t)byte_align * chan_num) {
		packet.type = SR_DF_LOGIC;
		packet.payload = &logic;
		logic.unitsize = 0;  /* unused for LA_CROSS_DATA */
		logic.format = LA_CROSS_DATA;
		logic.length = pb->post_len;
		logic.data = pb->post_buf;
		sr_session_send(sdi, &packet);
	}

	return G_SOURCE_CONTINUE;
}

/* DSO file replay: reads O-<ch>/0 from zip into post_buf (byte-interleaved
 * [ch0,ch1,...]), applies per-channel vdiv scaling, sends SR_DF_DSO, and
 * updates measurement stats. The data is read once (single block) and
 * re-sent each tick until limit_samples is reached. */
SR_PRIV int demo_receive_data_dso_file(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_dso dso;
	struct sr_channel *ch_list[MAX_PROBE_NUM];
	struct demo_packet_buffer *pb;
	char file_name[32];
	unz_file_info64 info;
	int chan_num, ch_index;
	uint64_t post_buf_len;

	chan_num = demo_collect_enabled_channels(sdi, SR_CHANNEL_DSO,
		ch_list, MAX_PROBE_NUM);
	if (chan_num < 1) {
		sr_err("demo: no enabled DSO channels for file replay.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	post_buf_len = DSO_PACKET_LEN;
	pb = demo_ensure_packet_buffer(devc, post_buf_len);
	if (!pb) {
		sr_err("demo: packet buffer alloc failed.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* Load the DSO data from the zip on first call (load_data==TRUE).
	 * DSO .demo files contain a single block per channel (O-<ch>/0). */
	if (devc->load_data) {
		for (ch_index = 0; ch_index < chan_num; ch_index++) {
			snprintf(file_name, sizeof(file_name), "O-%d/0", ch_index);
			if (unzLocateFile(devc->archive, file_name, 0) != UNZ_OK) {
				sr_err("demo: can't locate zip entry '%s'.", file_name);
				sr_dev_acquisition_stop(sdi);
				return G_SOURCE_CONTINUE;
			}
			if (unzGetCurrentFileInfo64(devc->archive, &info,
					NULL, 0, NULL, 0, NULL, 0) != UNZ_OK) {
				sr_err("demo: unzGetCurrentFileInfo64 failed.");
				sr_dev_acquisition_stop(sdi);
				return G_SOURCE_CONTINUE;
			}
			if (ch_index == 0) {
				if (info.uncompressed_size * chan_num > pb->post_buf_len) {
					/* Resize post_buf to fit all channels' data. */
					g_free(pb->post_buf);
					pb->post_buf = g_malloc(info.uncompressed_size * chan_num);
					if (!pb->post_buf) {
						pb->post_buf_len = 0;
						sr_dev_acquisition_stop(sdi);
						return G_SOURCE_CONTINUE;
					}
					pb->post_buf_len = info.uncompressed_size * chan_num;
				}
				pb->block_data_len = info.uncompressed_size;
			} else if (info.uncompressed_size != pb->block_data_len) {
				sr_err("demo: DSO block size mismatch for '%s'.", file_name);
				sr_dev_acquisition_stop(sdi);
				return G_SOURCE_CONTINUE;
			}
			if (unzOpenCurrentFile(devc->archive) != UNZ_OK) {
				sr_err("demo: can't open zip entry '%s'.", file_name);
				sr_dev_acquisition_stop(sdi);
				return G_SOURCE_CONTINUE;
			}
			/* Read into post_buf at offset ch_index (byte-interleaved:
			 * [ch0_s0, ch1_s0, ch0_s1, ch1_s1, ...]). We read the whole
			 * channel block contiguously first, then de-interleave below. */
			{
				uint8_t *tmp = g_malloc(pb->block_data_len);
				if (!tmp) {
					unzCloseCurrentFile(devc->archive);
					sr_dev_acquisition_stop(sdi);
					return G_SOURCE_CONTINUE;
				}
				unzReadCurrentFile(devc->archive, tmp, pb->block_data_len);
				unzCloseCurrentFile(devc->archive);
				/* Scatter into interleaved post_buf: sample i goes to
				 * position i*chan_num + ch_index. */
				for (uint64_t i = 0; i < pb->block_data_len; i++) {
					((uint8_t *)pb->post_buf)[i * chan_num + ch_index] = tmp[i];
				}
				g_free(tmp);
			}
		}
		pb->post_len = pb->block_data_len * chan_num;
		/* Apply per-channel vdiv scaling once on load. Subsequent ticks
		 * re-send the already-scaled data. If vdiv/offset change mid-
		 * stream the user must restart acquisition to re-scale. */
		demo_dso_vdiv_scale(devc, (uint8_t *)pb->post_buf, pb->post_len);
		devc->load_data = FALSE;
	}

	/* Build and send the DSO packet. */
	dso.data = pb->post_buf;
	dso.num_samples = pb->post_len / chan_num;
	dso.trig_flag = 1;
	dso.trig_ch = 0;
	dso.en_ch_num = (uint8_t)chan_num;
	dso.sample_bits = devc->dso_unit_bits;
	dso.trig_offset = (int16_t)(dso.num_samples / 2);
	dso.packet_len = (uint32_t)pb->post_len;
	dso.samplerate_tog = (uint32_t)devc->cur_samplerate;

	packet.type = SR_DF_DSO;
	packet.payload = &dso;
	sr_session_send(sdi, &packet);

	/* Update measurement stats. */
	demo_dso_status_update(devc, (uint8_t *)pb->post_buf, pb->post_len);

	/* Single-shot: stop after one frame. Loop/instant handled by caller. */
	if (!devc->instant && !devc->loop_mode)
		sr_dev_acquisition_stop(sdi);

	return G_SOURCE_CONTINUE;
}

/* ANALOG file replay: reads A-0/0 from zip into data_buf (with vdiv scaling
 * applied per-channel), then sends a chunk cyclically each tick as
 * SR_DF_ANALOG. Stops acquisition when limit_samples is reached. */
SR_PRIV int demo_receive_data_analog_file(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct sr_channel *ch_list[MAX_PROBE_NUM];
	struct demo_packet_buffer *pb;
	char file_name[32];
	unz_file_info64 info;
	int chan_num, ret;
	uint64_t chunk_len, back_len, front_len;

	chan_num = demo_collect_enabled_channels(sdi, SR_CHANNEL_ANALOG,
		ch_list, MAX_PROBE_NUM);
	if (chan_num < 1) {
		sr_err("demo: no enabled analog channels for file replay.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* Load A-0/0 from zip on first call. Expand the cyclic byte pattern
	 * to total_samples length (per-channel), apply per-channel vdiv
	 * scaling, store in devc->data_buf. */
	if (devc->load_data) {
		uint8_t *cycle_data;
		uint64_t total_buf_len;
		uint64_t per_block_after_expand;
		uint64_t cur_l;
		int ch_idx;

		snprintf(file_name, sizeof(file_name), "A-0/0");
		if (unzLocateFile(devc->archive, file_name, 0) != UNZ_OK) {
			sr_err("demo: can't locate zip entry '%s'.", file_name);
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		if (unzGetCurrentFileInfo64(devc->archive, &info,
				NULL, 0, NULL, 0, NULL, 0) != UNZ_OK) {
			sr_err("demo: unzGetCurrentFileInfo64 failed.");
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		cycle_data = g_malloc(ANALOG_DATA_LEN_PER_CYCLE);
		if (!cycle_data) {
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		if (unzOpenCurrentFile(devc->archive) != UNZ_OK) {
			g_free(cycle_data);
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		ret = unzReadCurrentFile(devc->archive, cycle_data,
			ANALOG_DATA_LEN_PER_CYCLE);
		unzCloseCurrentFile(devc->archive);
		if (ret < 0) {
			g_free(cycle_data);
			sr_err("demo: read error for A-0/0.");
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}

		/* Expand: each cycle byte is repeated per_block_after_expand
		 * times, interleaved per channel. Mirrors old fork. */
		total_buf_len = (uint64_t)(ANALOG_CYCLE_RATIO *
			(double)devc->total_samples * chan_num);
		if (total_buf_len % ANALOG_DATA_LEN_PER_CYCLE != 0)
			total_buf_len = total_buf_len / ANALOG_DATA_LEN_PER_CYCLE
				* ANALOG_DATA_LEN_PER_CYCLE;
		g_free(devc->data_buf);
		devc->data_buf = g_malloc0(total_buf_len);
		if (!devc->data_buf) {
			g_free(cycle_data);
			sr_dev_acquisition_stop(sdi);
			return G_SOURCE_CONTINUE;
		}
		devc->data_buf_len = total_buf_len;
		per_block_after_expand = total_buf_len / ANALOG_DATA_LEN_PER_CYCLE;

		/* Apply per-channel vdiv scaling on the cycle bytes and expand
		 * into data_buf with proper channel interleaving. */
		for (uint64_t i = 0; i < ANALOG_DATA_LEN_PER_CYCLE; i++) {
			uint8_t scaled = cycle_data[i];
			ch_idx = (int)(i % chan_num);
			demo_analog_vdiv_scale(devc, &scaled, 1, ch_idx);
			for (uint64_t j = 0; j < per_block_after_expand; j++) {
				if (i % chan_num == 0)
					cur_l = i * per_block_after_expand + j * chan_num;
				else
					cur_l = (i % chan_num) +
						(i / chan_num) * chan_num * per_block_after_expand +
						j * chan_num;
				if (cur_l < total_buf_len)
					((uint8_t *)devc->data_buf)[cur_l] = scaled;
			}
		}
		g_free(cycle_data);
		devc->load_data = FALSE;
		devc->packet_buffer = NULL;  /* will be allocated below */
	}

	/* Send a chunk cyclically from data_buf. */
	chunk_len = MIN(ANALOG_BUFSIZE, devc->data_buf_len);
	pb = demo_ensure_packet_buffer(devc, chunk_len);
	if (!pb) {
		sr_err("demo: analog packet buffer alloc failed.");
		sr_dev_acquisition_stop(sdi);
		return G_SOURCE_CONTINUE;
	}

	/* Track read position in pb->block_chan_read_pos (reused for analog). */
	if (pb->block_chan_read_pos + chunk_len >= devc->data_buf_len) {
		back_len = devc->data_buf_len - pb->block_chan_read_pos;
		front_len = chunk_len - back_len;
		memcpy(pb->post_buf,
			(uint8_t *)devc->data_buf + pb->block_chan_read_pos, back_len);
		memcpy((uint8_t *)pb->post_buf + back_len,
			devc->data_buf, front_len);
		pb->block_chan_read_pos = front_len;
	} else {
		memcpy(pb->post_buf,
			(uint8_t *)devc->data_buf + pb->block_chan_read_pos,
			chunk_len);
		pb->block_chan_read_pos += chunk_len;
	}

	/* Send SR_DF_ANALOG with byte encoding (uint8_t samples). */
	sr_analog_init(&analog, &encoding, &meaning, &spec, 0);
	encoding.unitsize = 1;
	encoding.is_signed = FALSE;
	encoding.is_float = FALSE;
	meaning.mq = SR_MQ_VOLTAGE;
	meaning.unit = SR_UNIT_VOLT;
	meaning.mqflags = SR_MQFLAG_AC;
	meaning.channels = NULL;
	for (int i = 0; i < chan_num; i++)
		meaning.channels = g_slist_append(meaning.channels, ch_list[i]);
	analog.data = pb->post_buf;
	analog.num_samples = chunk_len / chan_num;

	packet.type = SR_DF_ANALOG;
	packet.payload = &analog;
	sr_session_send(sdi, &packet);

	g_slist_free(meaning.channels);

	/* Stop when limit_samples is reached. */
	devc->sent_samples += chunk_len / chan_num;
	if (devc->limit_samples > 0 &&
	    devc->sent_samples >= devc->limit_samples) {
		sr_dev_acquisition_stop(sdi);
	}

	return G_SOURCE_CONTINUE;
}

