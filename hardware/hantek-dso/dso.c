/*
 * This file is part of the sigrok project.
 *
 * Copyright (C) 2012 Bert Vermeulen <bert@biot.com>
 * With protocol information from the hantekdso project,
 * Copyright (C) 2008 Oleg Khudyakov <prcoder@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "sigrok.h"
#include "sigrok-internal.h"
#include "config.h"
#include "dso.h"
#include <string.h>
#include <glib.h>
#include <libusb.h>

extern libusb_context *usb_context;
extern GSList *dev_insts;


static int send_begin(struct context *ctx)
{
	int ret;
	unsigned char buffer[] = {0x0f, 0x03, 0x03, 0x03, 0x68, 0xac, 0xfe,
	0x00, 0x01, 0x00};

	sr_dbg("hantek-dso: sending CTRL_BEGINCOMMAND");

	if ((ret = libusb_control_transfer(ctx->usb->devhdl,
			LIBUSB_REQUEST_TYPE_VENDOR, CTRL_BEGINCOMMAND,
			0, 0, buffer, sizeof(buffer), 200)) != sizeof(buffer)) {
		sr_err("failed to send begincommand: %d", ret);
		return SR_ERR;
	}

	return SR_OK;
}

static int send_bulkcmd(struct context *ctx, uint8_t *cmdstring, int cmdlen)
{
	int ret, tmp;

	if (send_begin(ctx) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(ctx->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, cmdlen, &tmp, 200)) != 0)
		return SR_ERR;

	return SR_OK;
}

SR_PRIV int dso_getmps(libusb_device *dev)
{
	struct libusb_device_descriptor des;
	struct libusb_config_descriptor *conf_dsc;
	const struct libusb_interface_descriptor *intf_dsc;
	int mps;

	if (libusb_get_device_descriptor(dev, &des) != 0)
		return 0;

	if (des.bNumConfigurations != 1)
		return 0;

	if (libusb_get_config_descriptor(dev, 0, &conf_dsc) != 0)
		return 0;

	mps = 0;
	intf_dsc = &(conf_dsc->interface[0].altsetting[0]);
	if (intf_dsc->bNumEndpoints != 2)
		goto err;

	if ((intf_dsc->endpoint[0].bEndpointAddress & 0x8f) !=
	    (2 | LIBUSB_ENDPOINT_OUT))
		/* The first endpoint should be 2 (outbound). */
		goto err;

	if ((intf_dsc->endpoint[1].bEndpointAddress & 0x8f) !=
	    (6 | LIBUSB_ENDPOINT_IN))
		/* The second endpoint should be 6 (inbound). */
		goto err;

	mps = intf_dsc->endpoint[1].wMaxPacketSize;

err:
	if (conf_dsc)
		libusb_free_config_descriptor(conf_dsc);

	return mps;
}

SR_PRIV int dso_open(int dev_index)
{
	libusb_device **devlist;
	struct libusb_device_descriptor des;
	struct sr_dev_inst *sdi;
	struct context *ctx;
	int err, skip, i;

	if (!(sdi = sr_dev_inst_get(dev_insts, dev_index)))
		return SR_ERR_ARG;
	ctx = sdi->priv;

	if (sdi->status == SR_ST_ACTIVE)
		/* already in use */
		return SR_ERR;

	skip = 0;
	libusb_get_device_list(usb_context, &devlist);
	for (i = 0; devlist[i]; i++) {
		if ((err = libusb_get_device_descriptor(devlist[i], &des))) {
			sr_err("hantek-dso: failed to get device descriptor: %d", err);
			continue;
		}

		if (des.idVendor != ctx->profile->fw_vid
		    || des.idProduct != ctx->profile->fw_pid)
			continue;

		if (sdi->status == SR_ST_INITIALIZING) {
			if (skip != dev_index) {
				/* Skip devices of this type that aren't the one we want. */
				skip += 1;
				continue;
			}
		} else if (sdi->status == SR_ST_INACTIVE) {
			/*
			 * This device is fully enumerated, so we need to find
			 * this device by vendor, product, bus and address.
			 */
			if (libusb_get_bus_number(devlist[i]) != ctx->usb->bus
				|| libusb_get_device_address(devlist[i]) != ctx->usb->address)
				/* this is not the one */
				continue;
		}

		if (!(err = libusb_open(devlist[i], &ctx->usb->devhdl))) {
			if (ctx->usb->address == 0xff)
				/*
				 * first time we touch this device after firmware upload,
				 * so we don't know the address yet.
				 */
				ctx->usb->address = libusb_get_device_address(devlist[i]);

			if(!(ctx->epin_maxpacketsize = dso_getmps(devlist[i])))
				sr_err("hantek-dso: wrong endpoint profile");
			else {
				sdi->status = SR_ST_ACTIVE;
				sr_info("hantek-dso: opened device %d on %d.%d interface %d",
					sdi->index, ctx->usb->bus,
					ctx->usb->address, USB_INTERFACE);
			}
		} else {
			sr_err("hantek-dso: failed to open device: %d", err);
		}

		/* if we made it here, we handled the device one way or another */
		break;
	}
	libusb_free_device_list(devlist, 1);

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR;

	return SR_OK;
}

SR_PRIV void dso_close(struct sr_dev_inst *sdi)
{
	struct context *ctx;

	ctx = sdi->priv;

	if (ctx->usb->devhdl == NULL)
		return;

	sr_info("hantek-dso: closing device %d on %d.%d interface %d", sdi->index,
		ctx->usb->bus, ctx->usb->address, USB_INTERFACE);
	libusb_release_interface(ctx->usb->devhdl, USB_INTERFACE);
	libusb_close(ctx->usb->devhdl);
	ctx->usb->devhdl = NULL;
	sdi->status = SR_ST_INACTIVE;

}

static int get_channel_offsets(struct context *ctx)
{
	int ret;

	sr_dbg("hantek-dso: getting channel offsets");

	ret = libusb_control_transfer(ctx->usb->devhdl,
			LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
			CTRL_READ_EEPROM, EEPROM_CHANNEL_OFFSETS, 0,
			(unsigned char *)&ctx->channel_levels,
			sizeof(ctx->channel_levels), 200);
	if (ret != sizeof(ctx->channel_levels)) {
		sr_err("failed to get channel offsets: %d", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_set_trigger_samplerate(struct context *ctx)
{
	int ret, tmp;
	uint8_t cmdstring[12];
	uint8_t timebasefast_small[] = {1, 2, 3, 4};
	uint8_t timebasefast_large[] = {0, 0, 2, 3};
	uint16_t timebase_small[] = { 0xffff, 0xfffc, 0xfff7, 0xffe8, 0xffce,
			0xff9c, 0xff07, 0xfe0d, 0xfc19, 0xf63d, 0xec79, 0xd8f1 };
	uint16_t timebase_large[] = { 0xffff, 0x0000, 0xfffc, 0xfff7, 0xffe8,
			0xffce, 0xff9d, 0xff07, 0xfe0d, 0xfc19, 0xf63d, 0xec79 };

	sr_dbg("hantek-dso: sending CMD_SET_TRIGGER_SAMPLERATE");

	memset(cmdstring, 0, sizeof(cmdstring));
	/* Command */
	cmdstring[0] = CMD_SET_TRIGGER_SAMPLERATE;

	/* Trigger source */
	cmdstring[2] = (ctx->triggersource & 0x03) << 6;

	/* Frame size */
	cmdstring[2] |= (ctx->framesize == FRAMESIZE_SMALL ? 0x01 : 0x02) << 3;

	/* Timebase fast (no idea what this means) */
	if (ctx->timebase < TIME_20us)
		tmp = 0;
	else if (ctx->timebase > TIME_200us)
		tmp = 4;
	else {
		if (ctx->framesize == FRAMESIZE_SMALL)
			tmp = timebasefast_small[ctx->timebase - 1];
		else
			tmp = timebasefast_large[ctx->timebase - 1];
	}
	cmdstring[2] |= tmp & 0x07;
cmdstring[2] = 0x45;
	/* Enabled channels */
	tmp = (((ctx->ch2_enabled ? 1 : 0) << 1) + (ctx->ch1_enabled ? 1 : 0)) - 1;
	cmdstring[3] = tmp;

	/* TODO: Fast rates channel */
	cmdstring[3] |= 0 << 5;

	/* Trigger slope */
	cmdstring[3] |= (ctx->triggerslope ? 1 : 0) << 4;

	/* Timebase */
	if (ctx->timebase < TIME_100us)
		tmp = 0;
	else if (ctx->timebase > TIME_400ms)
		tmp = 0xffed;
	else {
		if (ctx->framesize == FRAMESIZE_SMALL)
			tmp = timebase_small[ctx->timebase - 3];
		else
			tmp = timebase_large[ctx->timebase - 3];
	}
tmp = 0xebff;
	cmdstring[6] = tmp & 0xff;
	cmdstring[7] = (tmp >> 8) & 0xff;

	/* Trigger position (time) */
	tmp = 0x77660 + ctx->triggerposition;
tmp = 0x7006f;
	cmdstring[8] = tmp & 0xff;
	cmdstring[9] = (tmp >> 8) & 0xff;
	cmdstring[10] = (tmp >> 16) & 0xff;

	if (send_begin(ctx) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(ctx->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, sizeof(cmdstring),
			&tmp, 100)) != 0) {
		sr_err("Failed to set trigger/samplerate: %d", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_set_filters(struct context *ctx)
{
	int ret, tmp;
	uint8_t cmdstring[8];

	sr_dbg("hantek-dso: sending CMD_SET_FILTERS");

	memset(cmdstring, 0, sizeof(cmdstring));
	cmdstring[0] = CMD_SET_FILTERS;
	cmdstring[1] = 0x0f;
	if (ctx->filter_ch1)
		cmdstring[2] |= 0x80;
	if (ctx->filter_ch2)
		cmdstring[2] |= 0x40;
	if (ctx->filter_trigger)
		cmdstring[2] |= 0x20;

	if (send_begin(ctx) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(ctx->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, sizeof(cmdstring),
			&tmp, 100)) != 0) {
		sr_err("Failed to set filters: %d", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_set_voltage(struct context *ctx)
{
	int ret, tmp;
	uint8_t cmdstring[8];

	sr_dbg("hantek-dso: sending CMD_SET_VOLTAGE");

	memset(cmdstring, 0, sizeof(cmdstring));
	cmdstring[0] = CMD_SET_VOLTAGE;
	cmdstring[1] = 0x0f;
	cmdstring[2] = 0x03;
	cmdstring[2] |= ((2 - ctx->voltage_ch1 % 3) << 6);
	cmdstring[2] |= ((2 - ctx->voltage_ch2 % 3) << 4);
cmdstring[2] = 0x30;

	if (send_begin(ctx) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(ctx->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, sizeof(cmdstring),
			&tmp, 100)) != 0) {
		sr_err("Failed to set voltage: %d", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_set_relays(struct context *ctx)
{
	int ret, cv1, cv2;
	uint8_t relays[] = { 0x00, 0x04, 0x08, 0x02, 0x20, 0x40, 0x10, 0x01,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	sr_dbg("hantek-dso: sending CTRL_SETRELAYS");

	cv1 = ctx->voltage_ch1 / 3;
	cv2 = ctx->voltage_ch2 / 3;
relays[0] = 0x01;
	if (cv1 > 0)
		relays[1] = ~relays[1];

	if (cv1 > 1)
		relays[2] = ~relays[2];

	if (ctx->coupling_ch1 != COUPLING_AC)
		relays[3] = ~relays[3];

	if (cv2 > 0)
		relays[4] = ~relays[4];

	if (cv2 > 1)
		relays[5] = ~relays[5];

	if (ctx->coupling_ch2 != COUPLING_AC)
		relays[6] = ~relays[6];

	if (ctx->triggersource == TRIGGER_EXT || ctx->triggersource == TRIGGER_EXT10)
		relays[7] = ~relays[7];

	if ((ret = libusb_control_transfer(ctx->usb->devhdl,
			LIBUSB_REQUEST_TYPE_VENDOR, CTRL_SETRELAYS,
			0, 0, relays, sizeof(relays), 100)) != sizeof(relays)) {
		sr_err("failed to set relays: %d", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_set_voffsets(struct context *ctx)
{
	int offset, ret;
	uint16_t *ch_levels;
//	uint8_t offsets[17];
uint8_t offsets[] = {0x20,0x75,0x30,0x3f,0x20,0xbd,0x3f,0x02,0x20,0x00,0x71,0x01,0x2e,0x0b,0x3f,0x02,0x50};
//uint8_t offsets[] = {0xff, 0x75, 0x30, 0x3f, 0x20, 0xbd,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
//};

	sr_dbg("hantek-dso: sending CTRL_SETOFFSET");

//	memset(offsets, 0, sizeof(offsets));
//	/* Channel 1 */
//	ch_levels = ctx->channel_levels[0][VOLTAGE_10mV - ctx->voltage_ch1];
//	offset = (ch_levels[1] - ch_levels[0]) * ctx->voffset_ch1 + ch_levels[0];
//	offsets[0] = (offset >> 8) | 0x20;
//	offsets[1] = offset & 0xff;
//
//	/* Channel 2 */
//	ch_levels = ctx->channel_levels[1][VOLTAGE_10mV - ctx->voltage_ch2];
//	offset = (ch_levels[1] - ch_levels[0]) * ctx->voffset_ch2 + ch_levels[0];
//	offsets[2] = (offset >> 8) | 0x20;
//	offsets[3] = offset & 0xff;
//
//	/* Trigger */
//	offset = MAX_VERT_TRIGGER * ctx->voffset_trigger;
//	offsets[4] = (offset >> 8) | 0x20;
//	offsets[5] = offset & 0xff;

	if ((ret = libusb_control_transfer(ctx->usb->devhdl,
			LIBUSB_REQUEST_TYPE_VENDOR, CTRL_SETOFFSET,
			0, 0, offsets, sizeof(offsets), 100)) != sizeof(offsets)) {
		sr_err("failed to set offsets: %d", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_enable_trigger(struct context *ctx)
{
	int ret, tmp;
	uint8_t cmdstring[2];

	sr_dbg("hantek-dso: sending CMD_ENABLE_TRIGGER");

	memset(cmdstring, 0, sizeof(cmdstring));
	cmdstring[0] = CMD_ENABLE_TRIGGER;
	cmdstring[1] = 0x00;

	if (send_begin(ctx) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(ctx->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, sizeof(cmdstring),
			&tmp, 100)) != 0) {
		sr_err("Failed to enable trigger: %d", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_force_trigger(struct context *ctx)
{
	int ret, tmp;
	uint8_t cmdstring[2];

	sr_dbg("hantek-dso: sending CMD_FORCE_TRIGGER");

	memset(cmdstring, 0, sizeof(cmdstring));
	cmdstring[0] = CMD_FORCE_TRIGGER;
	cmdstring[1] = 0x00;

	if (send_begin(ctx) != SR_OK)
		return SR_ERR;

	if ((ret = libusb_bulk_transfer(ctx->usb->devhdl,
			DSO_EP_OUT | LIBUSB_ENDPOINT_OUT,
			cmdstring, sizeof(cmdstring),
			&tmp, 100)) != 0) {
		sr_err("Failed to force trigger: %d", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_init(struct context *ctx)
{

	sr_dbg("hantek-dso: initializing dso");

	if (get_channel_offsets(ctx) != SR_OK)
		return SR_ERR;

	if (dso_set_trigger_samplerate(ctx) != SR_OK)
		return SR_ERR;

	if (dso_set_filters(ctx) != SR_OK)
		return SR_ERR;

	if (dso_set_voltage(ctx) != SR_OK)
		return SR_ERR;

	if (dso_set_relays(ctx) != SR_OK)
		return SR_ERR;

	if (dso_set_voffsets(ctx) != SR_OK)
		return SR_ERR;

	if (dso_enable_trigger(ctx) != SR_OK)
		return SR_ERR;

	if (dso_force_trigger(ctx) != SR_OK)
		return SR_ERR;

	return SR_OK;
}

SR_PRIV uint8_t dso_get_capturestate(struct context *ctx)
{
	int ret, tmp;
	uint8_t cmdstring[2], inbuf[512];

	sr_dbg("hantek-dso: sending CMD_GET_CAPTURESTATE");

	cmdstring[0] = CMD_GET_CAPTURESTATE;
	cmdstring[1] = 0;

	if ((ret = send_bulkcmd(ctx, cmdstring, sizeof(cmdstring))) != SR_OK) {
		sr_dbg("Failed to send get_capturestate command: %d", ret);
		return CAPTURE_UNKNOWN;
	}

	if ((ret = libusb_bulk_transfer(ctx->usb->devhdl,
			DSO_EP_IN | LIBUSB_ENDPOINT_IN,
			inbuf, 512, &tmp, 100)) != 0) {
		sr_dbg("Failed to get capturestate: %d", ret);
		return CAPTURE_UNKNOWN;
	}

	return inbuf[0];
}

SR_PRIV uint8_t dso_capture_start(struct context *ctx)
{
	int ret;
	uint8_t cmdstring[2];

	sr_dbg("hantek-dso: sending CMD_CAPTURE_START");

	cmdstring[0] = CMD_CAPTURE_START;
	cmdstring[1] = 0;

	if ((ret = send_bulkcmd(ctx, cmdstring, sizeof(cmdstring))) != SR_OK) {
		sr_err("Failed to send capture_start command: %d", ret);
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dso_get_channeldata(struct context *ctx, libusb_transfer_cb_fn cb)
{
	struct libusb_transfer *transfer;
	int num_transfers, ret, i;
	uint8_t cmdstring[2];
	unsigned char *buf;

	sr_dbg("hantek-dso: sending CMD_GET_CHANNELDATA");

	cmdstring[0] = CMD_GET_CHANNELDATA;
	cmdstring[1] = 0;

	if ((ret = send_bulkcmd(ctx, cmdstring, sizeof(cmdstring))) != SR_OK) {
		sr_err("Failed to get channel data: %d", ret);
		return SR_ERR;
	}

	/* TODO: dso-2xxx only */
	num_transfers = ctx->framesize * sizeof(unsigned short) / ctx->epin_maxpacketsize;
	sr_dbg("hantek-dso: queueing up %d transfers", num_transfers);
	for (i = 0; i < num_transfers; i++) {
		if (!(buf = g_try_malloc(ctx->epin_maxpacketsize))) {
			sr_err("hantek-dso: %s: buf malloc failed", __func__);
			return SR_ERR_MALLOC;
		}
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, ctx->usb->devhdl,
				DSO_EP_IN | LIBUSB_ENDPOINT_IN, buf,
				ctx->epin_maxpacketsize, cb, ctx, 40);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("failed to submit transfer: %d", ret);
			/* TODO: Free them all. */
			libusb_free_transfer(transfer);
			g_free(buf);
			return SR_ERR;
		}
	}

	return SR_OK;
}
