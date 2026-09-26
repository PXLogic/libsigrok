/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
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

#include <config.h>
#include <string.h>
#include <stdlib.h>
#include <zip.h>
#include <errno.h>
#include <glib.h>
#include <glib/gstdio.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

/** @cond PRIVATE */
#define LOG_PREFIX "session-file"
/** @endcond */

/**
 * @file
 *
 * Loading and saving libsigrok session files.
 */

/**
 * @addtogroup grp_session
 *
 * @{
 */

/** @cond PRIVATE */
extern SR_PRIV struct sr_dev_driver session_driver;
/** @endcond */
static int session_driver_initialized = 0;

#if !HAVE_ZIP_DISCARD
/* Replacement for zip_discard() if it isn't available. */
/** @private */
SR_PRIV void sr_zip_discard(struct zip *archive)
{
	if (zip_unchange_all(archive) < 0 || zip_close(archive) < 0)
		sr_err("Failed to discard ZIP archive: %s", zip_strerror(archive));
}
#endif

/**
 * Read metadata entries from a session archive.
 *
 * @param[in] archive An open ZIP archive.
 * @param[in] entry Stat buffer filled in for the metadata archive member.
 *
 * @return A new key/value store containing the session metadata.
 *
 * @private
 */
SR_PRIV GKeyFile *sr_sessionfile_read_metadata(struct zip *archive,
			const struct zip_stat *entry)
{
	GKeyFile *keyfile;
	GError *error;
	struct zip_file *zf;
	char *metabuf;
	int metalen;

	if (entry->size > G_MAXINT || !(metabuf = g_try_malloc(entry->size))) {
		sr_err("Metadata buffer allocation failed.");
		return NULL;
	}
	zf = zip_fopen_index(archive, entry->index, 0);
	if (!zf) {
		sr_err("Failed to open metadata: %s", zip_strerror(archive));
		g_free(metabuf);
		return NULL;
	}
	metalen = zip_fread(zf, metabuf, entry->size);
	if (metalen < 0) {
		sr_err("Failed to read metadata: %s", zip_file_strerror(zf));
		zip_fclose(zf);
		g_free(metabuf);
		return NULL;
	}
	zip_fclose(zf);

	keyfile = g_key_file_new();
	error = NULL;
	g_key_file_load_from_data(keyfile, metabuf, metalen,
			G_KEY_FILE_NONE, &error);
	g_free(metabuf);

	if (error) {
		sr_err("Failed to parse metadata: %s", error->message);
		g_error_free(error);
		g_key_file_free(keyfile);
		return NULL;
	}
	return keyfile;
}

/** @private */
SR_PRIV int sr_sessionfile_check(const char *filename)
{
	struct zip *archive;
	struct zip_file *zf;
	struct zip_stat zs;
	uint64_t version;
	int ret;
	char s[11];
	GKeyFile *kf;
	GError *error;

	if (!filename)
		return SR_ERR_ARG;

	if (!g_file_test(filename, G_FILE_TEST_IS_REGULAR)) {
		sr_err("Not a regular file: %s.", filename);
		return SR_ERR;
	}

	if (!(archive = zip_open(filename, 0, NULL)))
		/* No logging: this can be used just to check if it's
		 * a sigrok session file or not. */
		return SR_ERR;

	/* check "version" (upstream sigrok format) */
	if ((zf = zip_fopen(archive, "version", 0))) {
		ret = zip_fread(zf, s, sizeof(s) - 1);
		zip_fclose(zf);
		if (ret < 0) {
			zip_discard(archive);
			return SR_ERR;
		}
		s[ret] = '\0';
		version = g_ascii_strtoull(s, NULL, 10);
		if (version == 0 || version > 2) {
			sr_dbg("Cannot handle sigrok session file version %" PRIu64 ".",
				version);
			zip_discard(archive);
			return SR_ERR;
		}
		sr_spew("Detected sigrok session file version %" PRIu64 ".", version);
		/* read "metadata" */
		if (zip_stat(archive, "metadata", 0, &zs) < 0) {
			sr_dbg("Not a valid sigrok session file.");
			zip_discard(archive);
			return SR_ERR;
		}
		zip_discard(archive);
		return SR_OK;
	}

	/* Try PXView v3 format: "header" file with [version] section */
	if (zip_stat(archive, "header", 0, &zs) >= 0) {
		kf = sr_sessionfile_read_metadata(archive, &zs);
		if (kf) {
			error = NULL;
			gchar *ver_str = g_key_file_get_string(kf, "version", "version", &error);
			if (ver_str && !error) {
				version = g_ascii_strtoull(ver_str, NULL, 10);
				g_free(ver_str);
				if (version >= 1 && version <= 3) {
					sr_spew("Detected PXView session file version %" PRIu64 ".", version);
					g_key_file_free(kf);
					zip_discard(archive);
					return SR_OK;
				}
			}
			if (error)
				g_error_free(error);
			g_key_file_free(kf);
		}
	}

	sr_dbg("Not a sigrok/PXView session file: no version found.");
	zip_discard(archive);
	return SR_ERR;
}

/** @private */
SR_PRIV struct sr_dev_inst *sr_session_prepare_sdi(const char *filename, struct sr_session **session)
{
	struct sr_dev_inst *sdi = NULL;

	sdi = g_malloc0(sizeof(struct sr_dev_inst));
	sdi->driver = &session_driver;
	sdi->status = SR_ST_INACTIVE;
	if (!session_driver_initialized) {
		/* first device, init the driver */
		session_driver_initialized = 1;
		sdi->driver->init(sdi->driver, NULL);
	}
	sr_dev_open(sdi);
	sr_session_dev_add(*session, sdi);
	(*session)->owned_devs = g_slist_append((*session)->owned_devs, sdi);
	sr_config_set(sdi, NULL, SR_CONF_SESSIONFILE,
			g_variant_new_string(filename));

	return sdi;
}

/**
 * Load the session from the specified filename.
 *
 * @param ctx The context in which to load the session.
 * @param filename The name of the session file to load.
 * @param session The session to load the file into.
 *
 * @retval SR_OK Success
 * @retval SR_ERR_MALLOC Memory allocation error
 * @retval SR_ERR_DATA Malformed session file
 * @retval SR_ERR This is not a session file
 */
SR_API int sr_session_load(struct sr_context *ctx, const char *filename,
		struct sr_session **session)
{
	GKeyFile *kf;
	GError *error;
	struct zip *archive;
	struct zip_stat zs;
	struct sr_dev_inst *sdi;
	struct sr_channel *ch;
	int ret, i, j;
	uint64_t tmp_u64;
	int total_channels, total_analog, k;
	int file_dev_mode = -1; /* device mode from header (LOGIC=0, DSO=1, ANALOG=2, MSO=3) */
	GSList *l;
	int unitsize = 0;
	char **sections, **keys, *val;
	char channelname[SR_MAX_CHANNELNAME_LEN + 1];
	gboolean file_has_logic;
	char *capturefile_name = NULL; /* capturefile of the logic device section */
	struct sr_dev_inst *cap_sdi = NULL; /* sdi owning capturefile_name */
	gboolean have_total_samples = FALSE; /* metadata had "total samples" */

	if ((ret = sr_sessionfile_check(filename)) != SR_OK)
		return ret;

	if (!(archive = zip_open(filename, 0, NULL)))
		return SR_ERR;

	/* Try upstream "metadata" first, then PXView "header" */
	if (zip_stat(archive, "metadata", 0, &zs) < 0) {
		if (zip_stat(archive, "header", 0, &zs) < 0) {
			sr_err("No metadata or header in session file.");
			zip_discard(archive);
			return SR_ERR;
		}
	}
	kf = sr_sessionfile_read_metadata(archive, &zs);
	zip_discard(archive);
	if (!kf)
		return SR_ERR_DATA;

	if ((ret = sr_session_new(ctx, session)) != SR_OK) {
		g_key_file_free(kf);
		return ret;
	}

	total_channels = 0;

	error = NULL;
	ret = SR_OK;
	file_has_logic = FALSE;
	sections = g_key_file_get_groups(kf, NULL);
	for (i = 0; sections[i] && ret == SR_OK; i++) {
		if (!strcmp(sections[i], "global") || !strcmp(sections[i], "version"))
			/* nothing really interesting in here yet */
			continue;
		if (!strncmp(sections[i], "device ", 7) || !strcmp(sections[i], "header")) {
			/* device section (upstream "device N" or PXView "header") */
			sdi = NULL;
			keys = g_key_file_get_keys(kf, sections[i], NULL, NULL);

			/* File contains analog data if there are analog channels. */
			total_analog = g_key_file_get_integer(kf, sections[i],
					"total analog",	&error);
			if (total_analog > 0 && !error)
				sdi = sr_session_prepare_sdi(filename, session);
			g_clear_error(&error);

			/* File contains logic data if a capturefile is set. */
			val = g_key_file_get_string(kf, sections[i],
				"capturefile", &error);
			if (val && !error) {
				if (!sdi)
					sdi = sr_session_prepare_sdi(filename, session);
				sr_config_set(sdi, NULL, SR_CONF_CAPTUREFILE,
						g_variant_new_string(val));
				g_free(capturefile_name);
				capturefile_name = g_strdup(val);
				cap_sdi = sdi;
				g_free(val);
				file_has_logic = TRUE;
			}
			g_clear_error(&error);

			/* Detect PXView 0-based probe naming (probe0 vs upstream probe1) */
			gboolean probe_zero_based = g_key_file_has_key(kf,
					sections[i], "probe0", NULL);

			/* Detect PXView 0-based analog naming (analog0 vs upstream analog1) */
			gboolean analog_zero_based = g_key_file_has_key(kf,
					sections[i], "analog0", NULL);

		for (j = 0; keys[j]; j++) {
			if (!strcmp(keys[j], "device mode")) {
				/* PXView writes "device mode = N" (LOGIC=0, DSO=1,
				 * ANALOG=2, MSO=3) so the file can be reopened in
				 * the correct work mode. Forward to session_driver
				 * via sr_config_set (int16 matches the key type
				 * registered in hwdriver.c). */
				int dev_mode = g_key_file_get_integer(kf, sections[i],
						keys[j], &error);
				if (sdi && !error && dev_mode >= 0) {
					file_dev_mode = dev_mode;
					sr_config_set(sdi, NULL, SR_CONF_DEVICE_MODE,
							g_variant_new_int16((int16_t)dev_mode));
				}
				g_clear_error(&error);
			} else if (!strcmp(keys[j], "samplerate")) {
					val = g_key_file_get_string(kf, sections[i],
							keys[j], &error);
					if (!sdi || !val || sr_parse_sizestring(val,
								&tmp_u64) != SR_OK) {
						g_free(val);
						ret = SR_ERR_DATA;
						break;
					}
					g_free(val);
					sr_config_set(sdi, NULL, SR_CONF_SAMPLERATE,
							g_variant_new_uint64(tmp_u64));
				} else if (!strcmp(keys[j], "total samples")) {
					/* PXView v3: "total samples = N" records the
					 * actual sample count saved in the file.
					 * Without this, the session_driver reports
					 * limit_samples=0, and the frontend falls back
					 * to its default sample limit (e.g. 1M),
					 * truncating loaded data to only a fraction
					 * of the original capture. */
					val = g_key_file_get_string(kf, sections[i],
							keys[j], &error);
					if (!sdi || !val || error) {
						g_free(val);
						g_clear_error(&error);
						/* non-fatal: old files may lack this key */
					} else {
						tmp_u64 = g_ascii_strtoull(val, NULL, 10);
						g_free(val);
						sr_config_set(sdi, NULL, SR_CONF_LIMIT_SAMPLES,
								g_variant_new_uint64(tmp_u64));
						have_total_samples = TRUE;
					}
				} else if (!strcmp(keys[j], "trigger pos")) {
					/* "trigger pos = N" — sample offset of the trigger
					 * point. Used by the frontend to position the trigger
					 * marker correctly when reopening a .pxl file. */
					val = g_key_file_get_string(kf, sections[i],
							keys[j], &error);
					if (sdi && val && !error) {
						tmp_u64 = g_ascii_strtoull(val, NULL, 10);
						sr_config_set(sdi, NULL, SR_CONF_TRIGGER_POS,
								g_variant_new_uint64(tmp_u64));
					}
					g_free(val);
					g_clear_error(&error);
				} else if (!strcmp(keys[j], "total blocks")) {
					/* "total blocks = N" — number of data chunks
					 * written to the file. Used by session_driver for
					 * block iteration. */
					val = g_key_file_get_string(kf, sections[i],
							keys[j], &error);
					if (sdi && val && !error) {
						tmp_u64 = g_ascii_strtoull(val, NULL, 10);
						sr_config_set(sdi, NULL, SR_CONF_NUM_BLOCKS,
								g_variant_new_uint64(tmp_u64));
					}
					g_free(val);
					g_clear_error(&error);
				} else if (!strcmp(keys[j], "trigger time")) {
					/* "trigger time = N" — milliseconds since Unix
					 * epoch, representing when the capture was taken.
					 * Written by StoreSession::meta_gen() in all modes.
					 * Exposed via SR_CONF_SESSION_TIME so the frontend
					 * can restore the original capture timestamp. */
					val = g_key_file_get_string(kf, sections[i],
							keys[j], &error);
					if (sdi && val && !error) {
						gint64 trig_time = (gint64)g_ascii_strtoll(val, NULL, 10);
						sr_config_set(sdi, NULL, SR_CONF_SESSION_TIME,
								g_variant_new_int64(trig_time));
					}
					g_free(val);
					g_clear_error(&error);
				} else if (!strcmp(keys[j], "hDiv")) {
					/* "hDiv = N" — DSO horizontal timebase (ns/div).
					 * Written in DSO mode by meta_gen(). */
					val = g_key_file_get_string(kf, sections[i],
							keys[j], &error);
					if (sdi && val && !error) {
						tmp_u64 = g_ascii_strtoull(val, NULL, 10);
						sr_config_set(sdi, NULL, SR_CONF_TIMEBASE,
								g_variant_new_uint64(tmp_u64));
					}
					g_free(val);
					g_clear_error(&error);
				} else if (!strcmp(keys[j], "bits")) {
					/* "bits = N" — ADC resolution in bits. Written in
					 * DSO and ANALOG modes by meta_gen(). */
					int bits_val = g_key_file_get_integer(kf, sections[i],
							keys[j], &error);
					if (sdi && !error && bits_val > 0) {
						sr_config_set(sdi, NULL, SR_CONF_UNIT_BITS,
								g_variant_new_byte((uint8_t)bits_val));
					}
					g_clear_error(&error);
				} else if (!strcmp(keys[j], "ref min")) {
					/* "ref min = N" — ADC reference minimum. Written in
					 * DSO and ANALOG modes by meta_gen(). */
					int ref_val = g_key_file_get_integer(kf, sections[i],
							keys[j], &error);
					if (sdi && !error) {
						sr_config_set(sdi, NULL, SR_CONF_REF_MIN,
								g_variant_new_uint32((uint32_t)ref_val));
					}
					g_clear_error(&error);
				} else if (!strcmp(keys[j], "ref max")) {
					/* "ref max = N" — ADC reference maximum. Written in
					 * DSO and ANALOG modes by meta_gen(). */
					int ref_val = g_key_file_get_integer(kf, sections[i],
							keys[j], &error);
					if (sdi && !error) {
						sr_config_set(sdi, NULL, SR_CONF_REF_MAX,
								g_variant_new_uint32((uint32_t)ref_val));
					}
					g_clear_error(&error);
				} else if (!strcmp(keys[j], "unitsize") && file_has_logic) {
					unitsize = g_key_file_get_integer(kf, sections[i],
							keys[j], &error);
					if (!sdi || unitsize <= 0 || error) {
						ret = SR_ERR_DATA;
						break;
					}
					sr_config_set(sdi, NULL, SR_CONF_CAPTURE_UNITSIZE,
							g_variant_new_uint64(unitsize));
				} else if (!strcmp(keys[j], "total probes")) {
				total_channels = g_key_file_get_integer(kf,
						sections[i], keys[j], &error);
				if (!sdi || total_channels < 0 || error) {
					ret = SR_ERR_DATA;
					break;
				}
				sr_config_set(sdi, NULL, SR_CONF_NUM_LOGIC_CHANNELS,
						g_variant_new_int32(total_channels));
				/* PXView v3 格式不含 unitsize，从 total probes 自动推算 */
				if (unitsize == 0 && total_channels > 0) {
					unitsize = (total_channels + 7) / 8;
					sr_config_set(sdi, NULL, SR_CONF_CAPTURE_UNITSIZE,
							g_variant_new_uint64(unitsize));
				}
				for (k = 0; k < total_channels; k++) {
						g_snprintf(channelname, sizeof(channelname),
								"%d", k);
						sr_channel_new(sdi, k, SR_CHANNEL_LOGIC,
								FALSE, channelname);
					}
			} else if (!strcmp(keys[j], "total analog")) {
				total_analog = g_key_file_get_integer(kf,
						sections[i], keys[j], &error);
				if (!sdi || total_analog < 0 || error) {
					ret = SR_ERR_DATA;
					break;
				}
				sr_config_set(sdi, NULL, SR_CONF_NUM_ANALOG_CHANNELS,
						g_variant_new_int32(total_analog));
				/* Determine channel type based on device mode.
				 * DSO mode (1): channels are SR_CHANNEL_DSO.
				 * ANALOG mode (2) / MSO mode (3): channels are SR_CHANNEL_ANALOG.
				 * Default: SR_CHANNEL_ANALOG (upstream compat). */
				int analog_ch_type = SR_CHANNEL_ANALOG;
				if (file_dev_mode < 0) {
					/* device mode key not yet parsed; read it directly */
					GError *e2 = NULL;
					int dm = g_key_file_get_integer(kf, sections[i],
							"device mode", &e2);
					if (!e2 && dm >= 0)
						file_dev_mode = dm;
					g_clear_error(&e2);
				}
				if (file_dev_mode == 1) /* DSO */
					analog_ch_type = SR_CHANNEL_DSO;
				for (k = total_channels; k < (total_channels + total_analog); k++) {
					g_snprintf(channelname, sizeof(channelname),
							"%d", k);
					sr_channel_new(sdi, k, analog_ch_type,
							FALSE, channelname);
				}
				} else if (!strncmp(keys[j], "probe", 5) && g_ascii_isdigit(keys[j][5])) {
				tmp_u64 = g_ascii_strtoull(keys[j] + 5, NULL, 10);
				if (!sdi || tmp_u64 > G_MAXINT) {
					ret = SR_ERR_DATA;
					break;
				}
				/* PXView uses 0-based probe indices, upstream uses 1-based */
				if (probe_zero_based) {
					/* PXView: probe indices may be non-contiguous (e.g. 0,2,5,7).
					 * Extend channel list if the index is beyond current size. */
					int cur_len = g_slist_length(sdi->channels);
					if (tmp_u64 >= (guint)cur_len) {
						for (k = cur_len; k <= (int)tmp_u64; k++) {
							g_snprintf(channelname, sizeof(channelname),
									"Logic Channel %d", k);
							sr_channel_new(sdi, k, SR_CHANNEL_LOGIC,
									FALSE, channelname);
						}
					}
					ch = g_slist_nth_data(sdi->channels, tmp_u64);
				} else {
					if (tmp_u64 == 0) {
						ret = SR_ERR_DATA;
						break;
					}
					ch = g_slist_nth_data(sdi->channels, tmp_u64 - 1);
				}
				if (!ch) {
					ret = SR_ERR_DATA;
					break;
				}
					val = g_key_file_get_string(kf, sections[i],
							keys[j], &error);
					if (!val) {
						ret = SR_ERR_DATA;
						break;
					}
					/* sr_session_save() */
					sr_dev_channel_name_set(ch, val);
					g_free(val);
					sr_dev_channel_enable(ch, TRUE);
				/* Only match analog<N> (N is a digit), not "analog bytes"/"analog float". */
			} else if (!strncmp(keys[j], "analog", 6) && g_ascii_isdigit(keys[j][6])) {
				tmp_u64 = g_ascii_strtoull(keys[j]+6, NULL, 10);
				if (!sdi || tmp_u64 > G_MAXINT) {
					ret = SR_ERR_DATA;
					break;
				}
				/* PXView 0-based: analog0 = first analog channel.
				 * Upstream 1-based: analog1 = first analog channel
				 * (looks for ch->index == 0, which is a logic channel —
				 * upstream bug, but we keep compat for 1-based files). */
				if (analog_zero_based) {
					/* 0-based: analog<N> → N-th analog channel,
					 * which has index total_channels + N */
					ch = NULL;
					int analog_idx = total_channels + (int)tmp_u64;
					for (l = sdi->channels; l; l = l->next) {
						ch = l->data;
						if ((guint64)ch->index == (guint64)analog_idx)
							break;
						else
							ch = NULL;
					}
				} else {
					/* 1-based upstream: analog<N> → ch->index == N-1 */
					if (tmp_u64 == 0) {
						ret = SR_ERR_DATA;
						break;
					}
					ch = NULL;
					for (l = sdi->channels; l; l = l->next) {
						ch = l->data;
						if ((guint64)ch->index == tmp_u64 - 1)
							break;
						else
							ch = NULL;
					}
				}
				if (!ch) {
					ret = SR_ERR_DATA;
					break;
				}
				val = g_key_file_get_string(kf, sections[i],
						keys[j], &error);
				if (!val) {
					ret = SR_ERR_DATA;
					break;
				}
				/* sr_session_save() */
				sr_dev_channel_name_set(ch, val);
				g_free(val);
				sr_dev_channel_enable(ch, TRUE);
			}
			}
			g_strfreev(keys);

			/* PXView v3: recalculate unitsize from actual channel count.
			 * total_probes is the count of *enabled* channels, but probe
			 * indices may be non-contiguous (e.g. 0,2,5,7), so the actual
			 * channel list can be larger. unitsize must cover all channels. */
			if (probe_zero_based && sdi && unitsize > 0) {
				int actual_channels = g_slist_length(sdi->channels);
				int calc_unitsize = (actual_channels + 7) / 8;
				if (calc_unitsize > unitsize) {
					unitsize = calc_unitsize;
					sr_config_set(sdi, NULL, SR_CONF_CAPTURE_UNITSIZE,
							g_variant_new_uint64(unitsize));
				}
			}
		}
	}

	/* Upstream sigrok session files (version 1/2 "metadata") never carry a
	 * "total samples" key. Without it the session_driver reports
	 * limit_samples=0 and the frontend falls back to its default sample
	 * limit (e.g. 1M), truncating the loaded capture to a fraction of the
	 * original (e.g. 1s of a 5s capture). Upstream PulseView applies no
	 * limit and simply streams the capture data until EOF; mirror that
	 * behavior here by computing the sample count from the actual size of
	 * the capture data (base file or <capturefile>-N chunks) / unitsize. */
	if (file_has_logic && capturefile_name && cap_sdi && unitsize > 0
			&& !have_total_samples) {
		struct zip *data_archive = zip_open(filename, 0, NULL);
		if (data_archive) {
			struct zip_stat dzs;
			uint64_t total_bytes = 0;
			if (zip_stat(data_archive, capturefile_name, 0, &dzs) != -1) {
				/* No chunks, just a single capture file. */
				total_bytes = dzs.size;
			} else {
				int chunk = 1;
				char chunkname[512];
				for (;; chunk++) {
					g_snprintf(chunkname, sizeof(chunkname), "%s-%d",
							capturefile_name, chunk);
					if (zip_stat(data_archive, chunkname, 0, &dzs) == -1)
						break;
					total_bytes += dzs.size;
				}
			}
			if (total_bytes > 0) {
				uint64_t computed = total_bytes / (uint64_t)unitsize;
				if (computed > 0) {
					sr_config_set(cap_sdi, NULL, SR_CONF_LIMIT_SAMPLES,
							g_variant_new_uint64(computed));
					sr_info("Session file has no 'total samples' key: "
							"computed %" PRIu64 " samples from capture "
							"data size (%" PRIu64 " bytes / unitsize %d).",
							computed, total_bytes, unitsize);
				}
			}
			zip_discard(data_archive);
		}
	}
	g_free(capturefile_name);

	g_strfreev(sections);
	g_key_file_free(kf);

	if (error) {
		sr_err("Failed to parse metadata: %s", error->message);
		g_error_free(error);
	}
	return ret;
}

/**
 * Load a session file and return the first device instance, without keeping
 * the sr_session alive. The caller owns the returned sdi and is responsible
 * for its lifecycle (adding it to its own session, freeing it when done).
 *
 * This is intended for applications (like PXView) that manage their own
 * sr_session but need libsigrok to parse the session file format, configure
 * channels, and set up the virtual session driver.
 *
 * Supports both upstream sigrok format (version/metadata + data-N chunks)
 * and PXView v3 format (header + L-<ch>/<n> per-channel chunks).
 *
 * @param ctx The context in which to load.
 * @param filename The session file to load.
 *
 * @return Pointer to a sr_dev_inst, or NULL on failure. The caller owns
 *         the returned instance.
 *
 * @since 0.6.0
 */
SR_API struct sr_dev_inst *sr_session_load_file_device(
		struct sr_context *ctx, const char *filename)
{
	struct sr_session *session = NULL;
	struct sr_dev_inst *sdi = NULL;

	if (sr_session_load(ctx, filename, &session) != SR_OK || !session) {
		sr_err("Failed to load session file '%s'.", filename);
		return NULL;
	}

	if (!session->devs) {
		sr_err("Session file '%s' contains no devices.", filename);
		sr_session_destroy(session);
		return NULL;
	}

	sdi = session->devs->data;

	/* Detach the sdi from the session's ownership so that
	 * sr_session_destroy() won't free it. The caller takes ownership. */
	session->owned_devs = g_slist_remove(session->owned_devs, sdi);

	/* Destroy the session container. This frees the devs list and
	 * remaining owned_devs, but our sdi was already detached above. */
	sr_session_destroy(session);

	sr_info("Loaded session file device '%s' (driver: %s).",
		filename, sdi->driver ? sdi->driver->name : "unknown");

	return sdi;
}

/** @} */
