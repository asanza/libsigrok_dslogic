/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2015 Uwe Hermann <uwe@hermann-uwe.de>
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <string.h>
#include "libsigrok.h"
#include "libsigrok-internal.h"

#define LOG_PREFIX "transform/invert"

static int receive(const struct sr_transform *t,
		struct sr_datafeed_packet *packet_in,
		struct sr_datafeed_packet **packet_out)
{
	const struct sr_datafeed_logic *logic;
	const struct sr_datafeed_analog *analog;
	struct sr_channel *ch;
	GSList *l;
	float *fdata, *f;
	int si, num_channels, c;
	uint8_t *b;
	uint64_t i, j;

	if (!t || !t->sdi || !packet_in || !packet_out)
		return SR_ERR_ARG;

	switch (packet_in->type) {
	case SR_DF_LOGIC:
		logic = packet_in->payload;
		for (i = 0; i <= logic->length - logic->unitsize; i += logic->unitsize) {
			for (j = 0; j < logic->unitsize; j++) {
				/* For now invert every bit in every byte. */
				b = (uint8_t *)logic->data + i + logic->unitsize - 1 - j;
				*b = ~(*b);
			}
		}
		break;
	case SR_DF_ANALOG:
		analog = packet_in->payload;
		fdata = (float *)analog->data;
		num_channels = g_slist_length(analog->channels);
		for (si = 0; si < analog->num_samples; si++) {
			/* For now invert all values in all channels. */
			for (l = analog->channels, c = 0; l; l = l->next, c++) {
				ch = l->data;
				(void)ch;
				f = &fdata[si * num_channels + c];
				*f = 1.0 / *f;
			}
		}
		break;
	default:
		sr_spew("Unsupported packet type %d, ignoring.", packet_in->type);
		break;
	}

	/* Return the in-place-modified packet. */
	*packet_out = packet_in;

	return SR_OK;
}

SR_PRIV struct sr_transform_module transform_invert = {
	.id = "invert",
	.name = "Invert",
	.desc = "Invert values",
	.options = NULL,
	.init = NULL,
	.receive = receive,
	.cleanup = NULL,
};
