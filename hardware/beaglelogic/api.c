/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2014 Kumar Abhishek <abhishek@theembeddedkitchen.net>
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

#include "protocol.h"
#include "beaglelogic.h"

SR_PRIV struct sr_dev_driver beaglelogic_driver_info;
static struct sr_dev_driver *di = &beaglelogic_driver_info;

/* Hardware options */
static const int32_t hwopts[] = {
	SR_CONF_CONN,
};

/* Hardware capabiities */
static const int32_t hwcaps[] = {
	SR_CONF_LOGIC_ANALYZER,
	SR_CONF_SAMPLERATE,
	SR_CONF_TRIGGER_MATCH,

	SR_CONF_LIMIT_SAMPLES,
	SR_CONF_CONTINUOUS,
	/* SR_CONF_EXTERNAL_CLOCK, TODO in a future BeagleLogic firmware */

	SR_CONF_NUM_LOGIC_CHANNELS,
};

/* Trigger matching capabilities */
static const int32_t soft_trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
	SR_TRIGGER_FALLING,
	SR_TRIGGER_EDGE,
};

/* Channels are numbered 0-31 (on the PCB silkscreen). */
SR_PRIV const char *beaglelogic_channel_names[NUM_CHANNELS + 1] = {
	"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12",
	"13", NULL,
};

/* Possible sample rates : 10 Hz to 100 MHz = (100 / x) MHz */
static const uint64_t samplerates[] = {
		SR_HZ(10),
		SR_MHZ(100),
		SR_HZ(1),
};

static int init(struct sr_context *sr_ctx)
{
	return std_init(sr_ctx, di, LOG_PREFIX);
}

static struct dev_context * beaglelogic_devc_alloc(void)
{
	struct dev_context *devc;

	/* Allocate the zeroed structure */
	if (!(devc = g_try_malloc0(sizeof(*devc)))) {
		sr_err("Device context alloc failed.");
		return NULL;
	}

	/* Default non-zero values (if any) */
	devc->fd = -1;
	devc->limit_samples = (uint64_t)-1;

	return devc;
}

static GSList *scan(GSList *options)
{
	struct drv_context *drvc;
	GSList *devices;
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct sr_channel *ch;
	int i, fd, maxch;

	(void)options;

	devices = NULL;
	drvc = di->priv;
	drvc->instances = NULL;

	/* Probe for /dev/beaglelogic */
	if (!g_file_test(BEAGLELOGIC_DEV_NODE, G_FILE_TEST_EXISTS))
		return NULL;

	/* Get a little information from BeagleLogic */
	if ((fd = beaglelogic_open()) == -1)
		return NULL;
	beaglelogic_get_sampleunit(fd, (uint32_t *)&i);
	beaglelogic_close(fd);
	maxch = (i == BL_SAMPLEUNIT_8_BITS) ? 8 : NUM_CHANNELS;

	sdi = sr_dev_inst_new(0, SR_ST_INACTIVE, NULL, "BeagleLogic", "1.0");
	sdi->driver = di;

	/* Signal */
	sr_info("BeagleLogic device found at "BEAGLELOGIC_DEV_NODE);

	/* Fill the channels */
	for (i = 0; i < maxch; i++) {
		if (!(ch = sr_channel_new(i, SR_CHANNEL_LOGIC, TRUE,
				beaglelogic_channel_names[i])))
			return NULL;
		sdi->channels = g_slist_append(sdi->channels, ch);
	}

	/* Allocate the device context */
	devc = beaglelogic_devc_alloc();
	sdi->priv = devc;

	drvc->instances = g_slist_append(drvc->instances, sdi);
	devices = g_slist_append(devices, sdi);

	return devices;
}

static GSList *dev_list(void)
{
	return ((struct drv_context *)(di->priv))->instances;
}

static int dev_clear(void)
{
	return std_dev_clear(di, NULL);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	int fd;
	struct dev_context *devc = sdi->priv;

	/* Open BeagleLogic */
	fd = beaglelogic_open_nonblock();
	if (fd == -1)
		return SR_ERR;

	/* Set fd and local attributes */
	devc->fd = fd;
	devc->pollfd.fd = fd;
	devc->pollfd.events = G_IO_IN;

	/* Get default attributes */
	beaglelogic_get_samplerate(fd, (uint32_t *)&devc->cur_samplerate);
	beaglelogic_get_sampleunit(fd, (void *)&devc->sampleunit);
	beaglelogic_get_triggerflags(fd, &devc->triggerflags);
	beaglelogic_get_buffersize(fd, &devc->buffersize);

	/* Buffer size and sample limit */
	devc->bufunitsize = beaglelogic_get_bufunitsize(fd);
	devc->limit_samples =
		devc->buffersize * SAMPLEUNIT_TO_BYTES(devc->sampleunit);

	/* Map the kernel capture FIFO for reads, saves 1 level of memcpy */
	if ((devc->sample_buf = beaglelogic_mmap(fd)) == MAP_FAILED) {
		sr_err("Unable to map capture buffer");
		beaglelogic_close(fd);
		return SR_ERR;
	}

	/* We're good to go now */
	sdi->status = SR_ST_ACTIVE;
	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	if (sdi->status == SR_ST_ACTIVE) {
		/* Close the memory mapping and the file */
		beaglelogic_munmap(devc->fd, devc->sample_buf);
		beaglelogic_close(devc->fd);
	}
	sdi->status = SR_ST_INACTIVE;
	return SR_OK;
}

static int cleanup(void)
{
	struct drv_context *drvc;
	struct sr_dev_inst *sdi;
	GSList *l;

	/* unused driver */
	if (!(drvc = di->priv))
		return SR_OK;

	/* Clean up the instances */
	for (l = drvc->instances; l; l = l->next) {
		sdi = l->data;
		di->dev_close(sdi);
		g_free(sdi->priv);
		sr_dev_inst_free(sdi);
	}
	g_slist_free(drvc->instances);
	drvc->instances = NULL;

	di->priv = NULL;

	return SR_OK;
}

static int config_get(int key, GVariant **data, const struct sr_dev_inst *sdi,
		const struct sr_channel_group *cg)
{
	struct dev_context *devc = sdi->priv;
	(void)cg;

	switch (key) {
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;

	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;

	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

/* Define maximum possible feasible buffer size on the BeagleBone Black
 * May be extended to >= 300 MB, but may leave system unstable
 * Change this to 128 MB if compiling for the BeagleBone White */
#define MAX_MEM		(256 * 1024 * 1024)

static int config_set(int key, GVariant *data, const struct sr_dev_inst *sdi,
		const struct sr_channel_group *cg)
{
	struct dev_context *devc = sdi->priv;
	uint64_t tmp_u64;
	(void)cg;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		tmp_u64 = g_variant_get_uint64(data);
		if (beaglelogic_set_samplerate(devc->fd, tmp_u64))
			return SR_ERR;
		devc->cur_samplerate = tmp_u64;
		break;

	case SR_CONF_LIMIT_SAMPLES:
		tmp_u64 = g_variant_get_uint64(data);
		devc->limit_samples = tmp_u64;

		/* Check if we have sufficient buffer size */
		tmp_u64 *= SAMPLEUNIT_TO_BYTES(devc->sampleunit);
		beaglelogic_set_triggerflags(devc->fd, BL_TRIGGERFLAGS_ONESHOT);
#if 0
		/* Try to allocate that buffer statically if possible. If not, then
		 * set continous mode on and fall back to 64 MB buffers */
		if (tmp_u64 <= MAX_MEM && devc->buffersize < tmp_u64) {
			beaglelogic_munmap(devc->fd, devc->sample_buf);

			if (beaglelogic_set_buffersize(devc->fd, tmp_u64)) {
				/* Restore previous state */
				beaglelogic_set_buffersize(devc->fd,
						devc->buffersize);
				beaglelogic_set_triggerflags(devc->fd,
						BL_TRIGGERFLAGS_CONTINUOUS);
				return SR_ERR;
			}
			/* Remap sample buffer */
			devc->sample_buf = beaglelogic_mmap(devc->fd);
			beaglelogic_get_buffersize(devc->fd, &devc->buffersize);

		} else {
			/* Use 64 MB buffers and continuous mode */
			sr_warn("insufficient memory available, check kernel "\
					"logs to see if any buffer was "\
					"dropped during continuous acquisition.");

			beaglelogic_set_buffersize(devc->fd, 64 * 1024 * 1024);
			beaglelogic_set_triggerflags(devc->fd,
					BL_TRIGGERFLAGS_CONTINUOUS);
		}
#endif
		break;

	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(int key, GVariant **data, const struct sr_dev_inst *sdi,
		const struct sr_channel_group *cg)
{
	int ret;
	GVariant *gvar;
	GVariantBuilder gvb;

	(void)sdi;
	(void)data;
	(void)cg;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
		*data = g_variant_new_fixed_array(G_VARIANT_TYPE_INT32,
				hwopts, ARRAY_SIZE(hwopts), sizeof(int32_t));
		break;
	case SR_CONF_DEVICE_OPTIONS:
		*data = g_variant_new_fixed_array(G_VARIANT_TYPE_INT32,
				hwcaps, ARRAY_SIZE(hwcaps), sizeof(int32_t));
		break;
	case SR_CONF_SAMPLERATE:
		g_variant_builder_init(&gvb, G_VARIANT_TYPE("a{sv}"));
		gvar = g_variant_new_fixed_array(G_VARIANT_TYPE("t"),
			samplerates, ARRAY_SIZE(samplerates), sizeof(uint64_t));
		g_variant_builder_add(&gvb, "{sv}", "samplerate-steps", gvar);
		*data = g_variant_builder_end(&gvb);
		break;
	case SR_CONF_TRIGGER_MATCH:
		*data = g_variant_new_fixed_array(G_VARIANT_TYPE_INT32,
				soft_trigger_matches, ARRAY_SIZE(soft_trigger_matches),
				sizeof(int32_t));
		break;
	default:
		return SR_ERR_NA;
	}

	return ret;
}

/* get a sane timeout for poll() */
#define BUFUNIT_TIMEOUT_MS(x)	(100 + (uint32_t)((x->bufunitsize * 1000) /  \
					(x->cur_samplerate)))

static int dev_acquisition_start(const struct sr_dev_inst *sdi,
				    void *cb_data)
{
	(void)cb_data;
	struct dev_context *devc = sdi->priv;
	struct sr_channel *ch;
	struct sr_trigger *trigger;
	GSList *l;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	/* Save user pointer */
	devc->cb_data = cb_data;

	/* Configure channels */
	beaglelogic_set_sampleunit(devc->fd, BL_SAMPLEUNIT_8_BITS);
	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (ch->index >= 8 && ch->enabled) {
			beaglelogic_set_sampleunit(devc->fd,
					BL_SAMPLEUNIT_16_BITS);
			break;
		}
	}
	beaglelogic_get_sampleunit(devc->fd, &devc->sampleunit);

	/* Configure triggers & send header packet */
	if ((trigger = sr_session_trigger_get())) {
		devc->stl = soft_trigger_logic_new(sdi, trigger);
		devc->trigger_fired = FALSE;
	} else
		devc->trigger_fired = TRUE;
	std_session_send_df_header(cb_data, LOG_PREFIX);

	/* Trigger and add poll on file */
	beaglelogic_start(devc->fd);
	sr_session_source_add_pollfd(&devc->pollfd, -1,
			beaglelogic_receive_data, (void *)sdi);

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi, void *cb_data)
{
	struct dev_context *devc = sdi->priv;
	struct sr_datafeed_packet pkt;

	(void)cb_data;

	if (sdi->status != SR_ST_ACTIVE)
		return SR_ERR_DEV_CLOSED;

	/* Execute a stop on BeagleLogic */
	beaglelogic_stop(devc->fd);

	/* Remove session source and send EOT packet */
	sr_session_source_remove_pollfd(&devc->pollfd);
	pkt.type = SR_DF_END;
	pkt.payload = NULL;
	sr_session_send(sdi, &pkt);

	return SR_OK;
}

SR_PRIV struct sr_dev_driver beaglelogic_driver_info = {
	.name = "beaglelogic",
	.longname = "BeagleLogic",
	.api_version = 1,
	.init = init,
	.cleanup = cleanup,
	.scan = scan,
	.dev_list = dev_list,
	.dev_clear = dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.priv = NULL,
};
