/*
 * Copyright 2012 Luke Dashjr
 * Copyright 2012 Con Kolivas
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <strings.h>
#include <sys/time.h>
#include <unistd.h>

#include "config.h"

#include "fpgautils.h"
#include "miner.h"

#define BITFORCE_SLEEP_MS 3000
#define BITFORCE_TIMEOUT_MS 7000
#define BITFORCE_LONG_TIMEOUT_MS 15000
#define BITFORCE_CHECK_INTERVAL_MS 10
#define WORK_CHECK_INTERVAL_MS 50
#define MAX_START_DELAY_US 100000

struct device_api bitforce_api;

#define BFopen(devpath)  serial_open(devpath, 0, -1, true)

static void BFgets(char *buf, size_t bufLen, int fd)
{
	do
		--bufLen;
	while (likely(bufLen && read(fd, buf, 1) && (buf++)[0] != '\n'));

	buf[0] = '\0';
}

static ssize_t BFwrite(int fd, const void *buf, ssize_t bufLen)
{
	if ((bufLen) != write(fd, buf, bufLen))
		return 0;
	else
		return bufLen;
}

#define BFclose(fd) close(fd)

static bool bitforce_detect_one(const char *devpath)
{
	int fdDev = BFopen(devpath);
	struct cgpu_info *bitforce;
	char pdevbuf[0x100];
	char *s;

	applog(LOG_DEBUG, "BFL: Attempting to open %s", devpath);

	if (unlikely(fdDev == -1)) {
		applog(LOG_ERR, "BFL: Failed to open %s", devpath);
		return false;
	}

	BFwrite(fdDev, "ZGX", 3);
	BFgets(pdevbuf, sizeof(pdevbuf), fdDev);
	if (unlikely(!pdevbuf[0])) {
		applog(LOG_ERR, "BFL: Error reading (ZGX)");
		return 0;
	}

	BFclose(fdDev);
	if (unlikely(!strstr(pdevbuf, "SHA256"))) {
		applog(LOG_ERR, "BFL: Didn't recognise BitForce on %s", devpath);
		return false;
	}

	// We have a real BitForce!
	bitforce = calloc(1, sizeof(*bitforce));
	bitforce->api = &bitforce_api;
	bitforce->device_path = strdup(devpath);
	bitforce->deven = DEV_ENABLED;
	bitforce->threads = 1;
	bitforce->sleep_ms = BITFORCE_SLEEP_MS;

	if (likely((!memcmp(pdevbuf, ">>>ID: ", 7)) && (s = strstr(pdevbuf + 3, ">>>")))) {
		s[0] = '\0';
		bitforce->name = strdup(pdevbuf + 7);
	}
	
	mutex_init(&bitforce->device_mutex);

	return add_cgpu(bitforce);
}

static char bitforce_detect_auto()
{
	return (serial_autodetect_udev     (bitforce_detect_one, "BitFORCE*SHA256") ?:
		serial_autodetect_devserial(bitforce_detect_one, "BitFORCE_SHA256") ?:
		0);
}

static void bitforce_detect()
{
	serial_detect_auto(bitforce_api.dname, bitforce_detect_one, bitforce_detect_auto);
}

struct bitforce_state {
	const char*job_cmd;
	unsigned char job_data[69];
	ssize_t job_data_len;
	uint32_t job_max_nonce;

	uint32_t running_max_nonce;
	bool justqueue;

	char buf[0x100];
};

static void get_bitforce_statline_before(char *buf, struct cgpu_info *bitforce)
{
	float gt = bitforce->temp;

	if (gt > 0)
		tailsprintf(buf, "%5.1fC ", gt);
	else
		tailsprintf(buf, "       ", gt);
	tailsprintf(buf, "        | ");
}

static uint64_t bitforce_can_limit_work(__maybe_unused struct thr_info *thr)
{
	return thr->can_limit_work ? 0x08b241e0 : 0xffffffff;
}

// NOTE: Not used as thread_prepare anymore, to make startup faster
static bool bitforce_thread_prepare(struct thr_info *thr)
{
	struct cgpu_info *bitforce = thr->cgpu;
	int fdDev = BFopen(bitforce->device_path);
	struct timeval now;
	char pdevbuf[3];

	if (unlikely(fdDev == -1)) {
		applog(LOG_ERR, "BFL%i: Failed to open %s", bitforce->device_id, bitforce->device_path);
		return false;
	}

	struct bitforce_state *state;
	state = thr->cgpu_data = calloc(1, sizeof(struct bitforce_state));

	BFwrite(fdDev, "ZUX0", 4);
	if (unlikely(read(fdDev, pdevbuf, 3) != 3 || !pdevbuf[0])) {
badZUX:
		applog(LOG_ERR, "%s %u: Unexpected response to ZUX0", bitforce->api->name, bitforce->device_id);
		return false;
	}
	else if (!strncmp("ERR", pdevbuf, 3)) {
		// Old protocol, full nonce ranges only
		BFgets(pdevbuf, sizeof(pdevbuf), fdDev);

		state->job_cmd = "ZDX";
		state->job_data_len = 60;
		state->job_max_nonce = 0xffffffff;
		thr->can_limit_work = false;
	}
	else if (!strncmp(">>>", pdevbuf, 3)) {
		// New protocol, nonce range supported

		if (read(fdDev, pdevbuf, 2) != 2)
			goto badZUX;
		char lastc = (pdevbuf[1] == '>') ? '\n' : '>';
		int cl = 0;
		while (1) {
			if (read(fdDev, pdevbuf, 1) != 1)
				goto badZUX;
			if (cl == 3) {
				if (pdevbuf[0] == lastc)
					break;
				if (pdevbuf[0] != '>')
					cl = 0;
			}
			else {
				if (pdevbuf[0] == '>')
					++cl;
				else
					cl = 0;
			}
		}

		state->job_cmd = "ZPX";
		state->job_data_len = 68;
		thr->can_limit_work = true;
	}
	memset(&state->job_data[0], '>', 8);
	memset(&state->job_data[state->job_data_len-8], '>', 8);

	thr->job_idle_usec = 10000;
	thr->results_delayed = true;

	bitforce->device_fd = fdDev;

	applog(LOG_INFO, "BFL%i: Opened %s (%s)", bitforce->device_id, bitforce->device_path, state->job_cmd);
	gettimeofday(&now, NULL);
	get_datestamp(bitforce->init, &now);

	return true;
}

static void biforce_clear_buffer(struct cgpu_info *bitforce)
{
	int fdDev = bitforce->device_fd;
	char pdevbuf[0x100];

	applog(LOG_DEBUG, "BFL%i: Clearing read buffer", bitforce->device_id);

	mutex_lock(&bitforce->device_mutex);
	do {
		pdevbuf[0] = '\0';
		BFgets(pdevbuf, sizeof(pdevbuf), fdDev);
	} while (pdevbuf[0]);
	mutex_unlock(&bitforce->device_mutex);
}

void bitforce_init(struct cgpu_info *bitforce)
{
	char *devpath = bitforce->device_path;
	int fdDev = bitforce->device_fd;
	char pdevbuf[0x100];
	char *s;

	applog(LOG_WARNING, "BFL%i: Re-initalizing", bitforce->device_id);

	biforce_clear_buffer(bitforce);

	mutex_lock(&bitforce->device_mutex);
	if (fdDev)
		BFclose(fdDev);
	bitforce->device_fd = 0;

	fdDev = BFopen(devpath);
	if (unlikely(fdDev == -1)) {
		mutex_unlock(&bitforce->device_mutex);
		applog(LOG_ERR, "BFL%i: Failed to open %s", bitforce->device_id, devpath);
		return;
	}

	BFwrite(fdDev, "ZGX", 3);
	BFgets(pdevbuf, sizeof(pdevbuf), fdDev);
	
	if (unlikely(!pdevbuf[0])) {
		mutex_unlock(&bitforce->device_mutex);
		applog(LOG_ERR, "BFL%i: Error reading (ZGX)", bitforce->device_id);
		return;
	}

	if (unlikely(!strstr(pdevbuf, "SHA256"))) {
		mutex_unlock(&bitforce->device_mutex);
		applog(LOG_ERR, "BFL%i: Didn't recognise BitForce on %s returned: %s", bitforce->device_id, devpath, pdevbuf);
		return;
	}
	
	if (likely((!memcmp(pdevbuf, ">>>ID: ", 7)) && (s = strstr(pdevbuf + 3, ">>>")))) {
		s[0] = '\0';
		bitforce->name = strdup(pdevbuf + 7);
	}

	bitforce->device_fd = fdDev;
	bitforce->sleep_ms = BITFORCE_SLEEP_MS;
	mutex_unlock(&bitforce->device_mutex);
}

static long bitforce_read_temperature(struct thr_info*thr)
{
	struct cgpu_info *bitforce = thr->cgpu;
	int fdDev = bitforce->device_fd;
	char pdevbuf[0x100];
	char *s;

	mutex_lock(&bitforce->device_mutex);
	BFwrite(fdDev, "ZLX", 3);
	BFgets(pdevbuf, sizeof(pdevbuf), fdDev);
	mutex_unlock(&bitforce->device_mutex);

	if ((!strncasecmp(pdevbuf, "TEMP", 4)) && (s = strchr(pdevbuf + 4, ':'))) {
		float temp = strtof(s + 1, NULL);
		return (long)(temp * 0x100);
	}

	return 0;
}

static bool bitforce_get_temp(struct cgpu_info *bitforce)
{
	long temp = bitforce_read_temperature(bitforce->thr[0]);
	if (temp > 0) {
		bitforce->temp = (float)temp / 0x100;
		if (temp > 0x100 * bitforce->cutofftemp) {
			applog(LOG_WARNING, "BFL%i: Hit thermal cutoff limit, disabling!", bitforce->device_id);
			bitforce->deven = DEV_RECOVER;

			bitforce->device_last_not_well = time(NULL);
			bitforce->device_not_well_reason = REASON_DEV_THERMAL_CUTOFF;
			bitforce->dev_thermal_cutoff_count++;
		}
	}

	return true;
}

static bool bitforce_job_prepare(struct thr_info *thr, struct work*work, uint64_t __maybe_unused last_nonce)
{
	struct bitforce_state *state = thr->cgpu_data;
	memcpy(&state->job_data[8], work->midstate, 32);
	memcpy(&state->job_data[40], work->data + 64, 12);
	if (state->job_data_len == 68) {
		uint32_t first_nonce = work->blk.nonce;
		state->job_data[52] = (first_nonce >> 24);
		state->job_data[53] = (first_nonce >> 16) & 0xff;
		state->job_data[54] = (first_nonce >>  8) & 0xff;
		state->job_data[55] = (first_nonce >>  0) & 0xff;
		state->job_data[56] = ( last_nonce >> 24);
		state->job_data[57] = ( last_nonce >> 16) & 0xff;
		state->job_data[58] = ( last_nonce >>  8) & 0xff;
		state->job_data[59] = ( last_nonce >>  0) & 0xff;
		state->job_max_nonce = last_nonce - first_nonce;
		work->blk.nonce = last_nonce;
		if (last_nonce != 0xffffffff)
			++work->blk.nonce;
	}
	else
	work->blk.nonce = 0xffffffff;
	return true;
}

static void bitforce_job_start(struct thr_info *thr)
{
	struct cgpu_info *bitforce = thr->cgpu;
	struct bitforce_state *state = thr->cgpu_data;
	int fdDev = bitforce->device_fd;
	char pdevbuf[0x100];
	char *s;
	bool retry = false;

	thr->job_running = false;
	if (!fdDev)
		return;
re_send:
	mutex_lock(&bitforce->device_mutex);
	if (!BFwrite(fdDev, state->job_cmd, 3)) {
		applog(LOG_ERR, "Error writing to BitForce (%s)", state->job_cmd);
		return;
	}
	BFgets(pdevbuf, sizeof(pdevbuf), fdDev);
	if (!pdevbuf[0] || (pdevbuf[0] == 'B')) {
		mutex_unlock(&bitforce->device_mutex);
		bitforce->wait_ms += WORK_CHECK_INTERVAL_MS;
		usleep(WORK_CHECK_INTERVAL_MS * 1000);
		goto re_send;
	} else if (unlikely(pdevbuf[0] != 'O' || pdevbuf[1] != 'K')) {
		mutex_unlock(&bitforce->device_mutex);
		applog(LOG_ERR, "BFL%i: Error: Send work reports: %s", bitforce->device_id, pdevbuf);
		goto commFail;
	}

	if (!BFwrite(fdDev, state->job_data, state->job_data_len)) {
		applog(LOG_ERR, "Error writing to BitForce (job data)");
		return;
	}
	BFgets(pdevbuf, sizeof(pdevbuf), fdDev);
	mutex_unlock(&bitforce->device_mutex);

	if (opt_debug) {
		s = bin2hex(state->job_data, state->job_data_len);
		applog(LOG_DEBUG, "BFL%i: job data: %s", bitforce->device_id, s);
		free(s);
	}

	if (unlikely(!pdevbuf[0])) {
		applog(LOG_ERR, "BFL%i: Error: Send block data returned empty string", bitforce->device_id);
		goto commFail;
	}

	if (unlikely(pdevbuf[0] != 'O' || pdevbuf[1] != 'K')) {
		applog(LOG_ERR, "BFL%i: Error: Send block data reports: %s", bitforce->device_id, pdevbuf);
		goto commFail;
	}

	state->running_max_nonce = state->job_max_nonce;
	thr->job_running = true;
	bitforce->wait_ms = 0;

	if (state->job_cmd[1] != 'P') {
		thr->job_idle_usec = (2000 * bitforce->sleep_ms) / 3;
		state->justqueue = true;
	}

	return;

commFail:
	if (retry)
		return;

	applog(LOG_ERR, "BFL%i: Comms error", bitforce->device_id);
	bitforce->device_last_not_well = time(NULL);
	bitforce->device_not_well_reason = REASON_DEV_COMMS_ERROR;
	bitforce->dev_comms_error_count++;
	/* empty read buffer */
	biforce_clear_buffer(bitforce);
	
	retry = true;
	goto re_send;
}

static int64_t bitforce_job_get_results(struct thr_info*thr, __maybe_unused struct work*work)
{
	unsigned int delay_time_ms = BITFORCE_CHECK_INTERVAL_MS;
	struct cgpu_info *bitforce = thr->cgpu;
	struct bitforce_state *state = thr->cgpu_data;
	int fdDev = bitforce->device_fd;

	if (!fdDev)
		return -1;

	bitforce->wait_ms += thr->job_idle_usec / 1000;

	if (state->justqueue) {
		queue_request(thr, false);
		thr->job_idle_usec = (bitforce->sleep_ms * 1000) - thr->job_idle_usec;
		state->justqueue = false;
		return 0;
	}

	if (bitforce->wait_ms < BITFORCE_LONG_TIMEOUT_MS) {
		if (unlikely(work_restart[thr->id].restart))
			return 0;
		mutex_lock(&bitforce->device_mutex);
		BFwrite(fdDev, "ZFX", 3);
		BFgets(state->buf, sizeof(state->buf), fdDev);
		mutex_unlock(&bitforce->device_mutex);
		/* BFL does not respond during throttling */
		if (!(state->buf[0] && state->buf[0] != 'B')) {
			/* if BFL is throttling, no point checking so quickly */
			delay_time_ms = (state->buf[0] ? BITFORCE_CHECK_INTERVAL_MS : 2*WORK_CHECK_INTERVAL_MS);
			thr->job_idle_usec = delay_time_ms * 1000;
			return 0;
		}
	}

	applog(LOG_DEBUG, "BFL%i: waited %dms for %08llx hashes until %s", bitforce->device_id, bitforce->wait_ms, (unsigned long long)state->running_max_nonce, state->buf);
	thr->job_running = false;
	work->blk.nonce = 0xffffffff;
	if (state->buf[2] == '-') 
		;   /* No valid nonce found */
	else if (state->buf[0] == 'I') 
		return 0;          /* Device idle */
	else if (strncasecmp(state->buf, "NONCE-FOUND", 11)) {
		applog(LOG_WARNING, "BFL%i: Error: Get result reports: %s", bitforce->device_id, state->buf);
		return 0;
	}

	return (uint64_t)state->running_max_nonce + 1LL;
}

static int64_t bitforce_job_process_results(struct thr_info*thr, struct work*work)
{
	struct cgpu_info *bitforce = thr->cgpu;
	struct bitforce_state *state = thr->cgpu_data;

	unsigned int delay_time_ms;

	if (state->buf[0] == 'B')
		return 0;  // ignored

	if (bitforce->wait_ms >= BITFORCE_TIMEOUT_MS) {
		applog(LOG_ERR, "BFL%i: took longer than %dms", bitforce->device_id, BITFORCE_TIMEOUT_MS);
		bitforce->device_last_not_well = time(NULL);
		bitforce->device_not_well_reason = REASON_DEV_OVER_HEAT;
		bitforce->dev_over_heat_count++;
		if (!state->buf[0])           /* Only return if we got nothing after timeout - there still may be results */
            return 0;
	} else if (state->buf[0] == 'N') {/* Hashing complete (NONCE-FOUND or NO-NONCE) */
		    /* Simple timing adjustment */
	        delay_time_ms = bitforce->sleep_ms;
		if (bitforce->wait_ms > (bitforce->sleep_ms + BITFORCE_CHECK_INTERVAL_MS))
			bitforce->sleep_ms += (unsigned int) ((double) (bitforce->wait_ms - bitforce->sleep_ms) / 1.6);
		else if (bitforce->wait_ms == bitforce->sleep_ms)
			bitforce->sleep_ms -= WORK_CHECK_INTERVAL_MS;
		if (delay_time_ms != bitforce->sleep_ms)
			  applog(LOG_DEBUG, "BFL%i: Wait time changed to: %d", bitforce->device_id, bitforce->sleep_ms, bitforce->wait_ms);
	}

	char*pnoncebuf = &state->buf[12];
	uint32_t nonce;

	if (!pnoncebuf[0])
		return 0;  // ignored

	while (1) {
		hex2bin((void*)&nonce, pnoncebuf, 4);
#ifndef __BIG_ENDIAN__
		nonce = swab32(nonce);
#endif
		submit_nonce(thr, work, nonce);
		if (pnoncebuf[8] != ',')
			break;
		pnoncebuf += 9;
	}

	state->buf[12] = '\0';
	return 0;  // ignored
}

static void bitforce_shutdown(struct thr_info *thr)
{
	struct cgpu_info *bitforce = thr->cgpu;

	BFclose(bitforce->device_fd);
	bitforce->device_fd = 0;
}

static void biforce_thread_enable(struct thr_info *thr)
{
	struct cgpu_info *bitforce = thr->cgpu;

	bitforce_init(bitforce);
}

static bool bitforce_get_stats(struct cgpu_info *bitforce)
{
	return bitforce_get_temp(bitforce);
}

static bool bitforce_thread_init(struct thr_info *thr)
{
	struct cgpu_info *bitforce = thr->cgpu;
	unsigned int wait;

	if (!bitforce_thread_prepare(thr))
		return false;

	/* Pause each new thread a random time between 0-100ms 
	so the devices aren't making calls all at the same time. */
	wait = (rand() * MAX_START_DELAY_US)/RAND_MAX;
	applog(LOG_DEBUG, "BFL%i: Delaying start by %dms", bitforce->device_id, wait / 1000);
	usleep(wait);

	return true;
}

static struct api_data *bitforce_api_stats(struct cgpu_info *cgpu)
{
	struct api_data *root = NULL;

	// Warning, access to these is not locked - but we don't really
	// care since hashing performance is way more important than
	// locking access to displaying API debug 'stats'
	// If locking becomes an issue for any of them, use copy_data=true also
	root = api_add_uint(root, "Sleep Time", &(cgpu->sleep_ms), false);

	return root;
}

struct device_api bitforce_api = {
	.dname = "bitforce",
	.name = "BFL",
	.api_detect = bitforce_detect,
	.get_api_stats = bitforce_api_stats,
	.reinit_device = bitforce_init,
	.get_statline_before = get_bitforce_statline_before,
	.get_stats = bitforce_get_stats,
	.can_limit_work = bitforce_can_limit_work,
	.thread_init = bitforce_thread_init,
	.thread_shutdown = bitforce_shutdown,
	.thread_enable = biforce_thread_enable,
	.read_temperature = bitforce_read_temperature,
	.job_prepare = bitforce_job_prepare,
	.job_start = bitforce_job_start,
	.job_get_results = bitforce_job_get_results,
	.job_process_results = bitforce_job_process_results,
};
