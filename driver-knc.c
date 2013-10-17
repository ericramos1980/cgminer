/* cgminer driver for KnCminer Mars prototype */

/* Written ground up, references: modminer driver and spiflash-program */

#include <stdlib.h>

#include <ftdi.h>
#include <assert.h>

#define DO_SELFTEST 1

#if 1
#define CLOCK_IDLE 0
#define WRITE_FLANK MPSSE_WRITE_NEG
#define READ_FLANK 0
#else
#define CLOCK_IDLE 1
#define WRITE_FLANK 0
#define READ_FLANK MPSSE_READ_NEG
#endif

#include "logging.h"
#include "miner.h"

/* Actually number of cores permitted on one USB device */
#define MAX_CHIPS 16
#define JUPITER_FIRMWARE_VERSION 0xa001

enum fpga_commands {
	CMD_IDENTIFY = 0x80, CMD_SETSTATE = 0x81, CMD_HALT = 0x83, CMD_REPORT = 0x82
};

struct device_drv knc_drv;
struct core_state {
	struct work *active, *queued;
	uint8_t chip_id;
	uint16_t core_id;
	unsigned char active_slot, queued_slot, next_slot;
	unsigned char last_nonce_slot;
	uint32_t prev_progress, last_nonce;
};
struct core_status {
	int next_state;
	uint32_t nonce_checked;
	int active_slot;
	int nonce_slot;
	uint32_t golden_nonce;
};

struct knc_state {
	struct ftdi_context *ctx;
	int devices;
	struct core_state *cores;
};

extern struct work *make_work(void);

static void knc_discardactive(struct cgpu_info *cgpu, int dev);

static bool
knc_cs_low(struct ftdi_context *ctx)
{
	applog(LOG_DEBUG, "KnC CS LOW");
	const unsigned char command[] =
	{
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	};

	if (ftdi_write_data(ctx, (void *) command, sizeof command) != sizeof command)
		goto l_abort;
	return true;
      l_abort:
	return false;
}

static bool
knc_send_halt(struct ftdi_context *ctx, uint8_t chip_id, uint16_t core_id)
{
	const unsigned char halt_command[] =
	{
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    MPSSE_DO_WRITE | WRITE_FLANK, -1 + 1 + 1 + 2, 0, CMD_HALT, chip_id, core_id >> 8, core_id & 0xff,
	    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b
	};		// ~CS

	if (ftdi_write_data(ctx, (void *) halt_command, sizeof halt_command) != sizeof halt_command)
		goto l_abort;
	return true;
      l_abort:
	return false;
}

static int
knc_send_work(struct ftdi_context *ctx, uint8_t chip_id, uint16_t core_id, unsigned char slot, struct work *work)
{
	int i;
	// Send work (TODO: don't do this if there's no new work?)
	const unsigned char workcmd_template[] =
	{
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    MPSSE_DO_WRITE | WRITE_FLANK, -1 + 1 + 1 + 2 + 1 + 8 * 4 + 3 * 4, 0,
	    CMD_SETSTATE, chip_id, core_id >> 8, core_id & 0xff, slot | 0xf0, };
	const int midstate_offset = sizeof workcmd_template;
	unsigned char workcmd[sizeof(workcmd_template) + 8 * 4 + 3 * 4];

	memcpy(workcmd, workcmd_template, sizeof workcmd_template);
	for (i = 0; i < 8 * 4; i++)
		workcmd[midstate_offset + i] = work->midstate[8 * 4 - i - 1];
	for (i = 0; i < 3 * 4; i++)
		workcmd[midstate_offset + 8 * 4 + i] = work->data[16 * 4 + 3 * 4 - i - 1];
	knc_cs_low(ctx);
	if (ftdi_write_data(ctx, (void *) workcmd, sizeof workcmd)
	    != sizeof workcmd)
		goto l_abort;
	applog(LOG_INFO, "KnC status wrote new work %d:%d:%d (%lu byte)", chip_id, core_id, slot, (unsigned long)sizeof workcmd);
	for (i = 0; i < sizeof workcmd; i += 8) {
		applog(LOG_INFO, "KnC status wrote: %02x %02x %02x %02x %02x %02x %02x %02x",
		    workcmd[i], workcmd[i + 1], workcmd[i + 2], workcmd[i + 3],
		    workcmd[i + 4], workcmd[i + 5], workcmd[i + 6], workcmd[i + 7]);
	}
	knc_cs_low(ctx);
	return 0;
      l_abort:
	applog(LOG_ERR, "KnC failed to assign new work!\n");
	knc_cs_low(ctx);
	return -1;
}

static void
knc_assign_work(struct cgpu_info *cgpu, int dev,
    struct work *work)
{
	struct knc_state *knc = cgpu->knc_state;
	struct core_state *core = knc->cores + dev;
	unsigned char slot;

	// Get a new slot ID
	slot = (core->next_slot++) & 0x07;

	if (knc_send_work(knc->ctx, core->chip_id, core->core_id, slot, work) != 0)
		goto l_abort;

	core->queued = work;
	core->queued_slot = slot;
	return;

      l_abort:
	// Drop the work in the bitbucket, because we failed to send it
	work_completed(cgpu, work);
}

static int
knc_read_status(struct ftdi_context *ctx, uint8_t chip_id, uint16_t core_id, struct core_status *status)
{
	int len, i;
	uint8_t response[7];
	const unsigned char getstatuscmd[] =
	{
	    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
	    MPSSE_DO_WRITE | WRITE_FLANK, -1 + 1 + 1 + 2, 0, CMD_REPORT, chip_id, core_id >> 8, core_id & 0xff,
	    MPSSE_DO_READ | READ_FLANK, sizeof(response) - 1, 0,
	    SEND_IMMEDIATE
        };

	knc_cs_low(ctx);
	len = ftdi_write_data(ctx, (void *) getstatuscmd, sizeof getstatuscmd);
	applog(LOG_DEBUG, "KnC ftdi write: wrote %lu, got %d (requested %lu) "
		"%02x%02x%02x %02x%02x%02x %02x%02x%02x%02x %02x%02x%02x %02x",
		    (unsigned long)sizeof getstatuscmd, len, (unsigned long)sizeof(response),
			getstatuscmd[0],
			getstatuscmd[1],
			getstatuscmd[2],
			getstatuscmd[3],
			getstatuscmd[4],
			getstatuscmd[5],
			getstatuscmd[6],
			getstatuscmd[7],
			getstatuscmd[8],
			getstatuscmd[9],
			getstatuscmd[10],
			getstatuscmd[11],
			getstatuscmd[12],
			getstatuscmd[13]
		);
	if (len != sizeof getstatuscmd) {
		goto l_abort;
	}
	for (i = 1, len = 0; (len < sizeof(response)) && i < 10; i++) {
		int r = ftdi_read_data(ctx, response + len, sizeof(response) - len);
		applog(LOG_DEBUG, "ftdi_read = %d", r);
		if (r < 0)
			goto l_abort;
		else
			len += r;
	}
	applog(LOG_DEBUG, "KnC read status: %02x:%04x %02x%02x%02x%02x%02x%02x%02x",
	    chip_id,
	    core_id,
	    response[0],
	    response[1],
	    response[2],
	    response[3],
	    response[4],
	    response[5],
	    response[6]);
	if (len != sizeof(response)) {
		applog(LOG_ERR, "KnC status read: too many tries, wanted %lu, got %d",
		    (unsigned long)sizeof(response), len);
		goto l_abort;
	}
	knc_cs_low(ctx);
	status->next_state = (response[0] & 1 << 5) != 0;
	status->nonce_checked = response[1] << 24;
	status->active_slot = response[2] >> 4;
	status->nonce_slot = response[2] & 0xf;
	status->golden_nonce = (response[3] << 24 | response[4] << 16 | response[5] << 8 | response[6] << 0);
	return 0;
      l_abort:
	knc_cs_low(ctx);
	return -1;
}

static int
knc_selftest(struct ftdi_context *ctx, struct core_state *cores, int devices)
{
	if (!DO_SELFTEST)
		return 1;

	// Code copied from icarus driver for verification
	const char golden_ob[] =

	"90f741afb3ab06f1a582c5c85ee7a561912b25a7cd09c060a89b3c2a73a48e22"
	//"73a48e22a89b3c2acd09c060912b25a75ee7a561a582c5c8b3ab06f190f741af" // wswap
	//"228ea4732a3c9ba860c009cda7252b9161a5e75ec8c582a5f106abb3af41f790" // both
	//"af41f790f106abb3c8c582a561a5e75ea7252b9160c009cd2a3c9ba8228ea473" // bswap

	"0000000000000000000000000000000000000000"

	"1571d1be4de695931a269421"
	//"1a2694214de695931571d1be"  // wswap
	//"2194261a9395e64dbed17115"  // both
	//"bed171159395e64d2194261a"  // bswap

	// nonce 

	// One hit: c9540e39  (cgminer order)
	//"4a548fe471fa3a9a1371144556c3f64d2500b4826008fe4bbf7698c94eba7946"  //1
	// reversed word order from cgminer order
	//"4eba7946bf7698c96008fe4b2500b48256c3f64d1371144571fa3a9a4a548fe4"   //2
	// byteswap from cgminer order
	//"e48f544a9a3afa71451471134df6c35682b400254bfe0860c99876bf4679ba4e"   //3
	// reversed byte order: reverse+byteswap
	//"4679ba4ec99876bf4bfe086082b400254df6c356451471139a3afa71e48f544a"   //4
	//"0000000000000000000000000000000000000000"
	//"ce22a72f4f6726141a0b3287" //1
	//"1a0b32874f672614ce22a72f" //2
	//"2fa722ce1426674f87320b1a" //3
	//"87320b1a1426674f2fa722ce" //4

	// hits for 1 1: c9540e39
	// hits for 2 2: f1c75c04
	// hits for 3 3: 6e033221 
	//          4 4  294deb91 d7771c00
	//          1 2

/*
 * // Two hits. Icarus order (string reversed from cgminer order).
 * "4679ba4ec99876bf4bfe086082b400254df6c356451471139a3afa71e48f544a"
 * "00000000000000000000000000000000"
 * "0000000087320b1a1426674f2fa722ce" */ ;

	const char golden_nonce_hex[] = "0e33337a";

	unsigned char ob_bin[64];
	char *nonce_hex;

	hex2bin(ob_bin, golden_ob, sizeof(ob_bin));
	struct work *testwork = make_work();
	memcpy(testwork->midstate, ob_bin, 32);
	memcpy(testwork->data + 16 * 4, ob_bin + 64 - 3 * 4, 3 * 4);
	
	int dev;
	int good_core[devices];
	int good_cores = 0;
	for (dev = 0; dev < devices; dev++) {
		struct core_status status;
		struct core_state *core = cores + dev;
		knc_read_status(ctx, core->chip_id, core->core_id, &status);
		if (dev && 0)
			sleep(2);
		knc_send_work(ctx, core->chip_id, core->core_id, 0xf - (status.nonce_slot == 0xf), testwork);
		good_core[dev] = 0;
	}

	unsigned char golden_nonce_bin[4];
	hex2bin((void *)&golden_nonce_bin, golden_nonce_hex, sizeof(golden_nonce_bin));
	uint32_t golden_nonce;
	golden_nonce = golden_nonce_bin[0] << 24 | golden_nonce_bin[1] << 16 | golden_nonce_bin[2] << 8 | golden_nonce_bin[3] << 0;
	
	int i;
	for (i = 0; i < 400; i++) {
		for (dev = 0; dev < devices; dev++) {
			struct core_state *core = cores + dev;
			struct core_status status;
			knc_read_status(ctx, core->chip_id, core->core_id, &status);
			applog(LOG_DEBUG, "KnC init: Nonce: %08x (%08x expected)", status.golden_nonce, golden_nonce);
			if (status.golden_nonce == golden_nonce && status.nonce_slot == status.active_slot) {
				applog(LOG_DEBUG, "KnC init: Expected nonce found!");
				if (!good_core[dev]) {
					good_core[dev] = 1;
					good_cores++;
				}
			}
		}
		if (good_cores == devices)
			break;
		sleep(1);
	}
	if (good_cores != devices) {
		applog(LOG_ERR, "KnC miner failed selftest");
		return false;
	}
	return true;
}


#define check(X) if ((X)<0) goto l_abort;
static bool
knc_detect_one(struct ftdi_context *ctx)
{

	// Set up MPSSE engine
	const unsigned char command[] =
	{
	    TCK_DIVISOR, 80, 0,
	    LOOPBACK_END
	};

	check(ftdi_usb_reset(ctx));
	check(ftdi_set_bitmode(ctx, 0, BITMODE_RESET));
	check(ftdi_usb_purge_buffers(ctx));
	check(ftdi_write_data_set_chunksize(ctx, 0x10000));
	check(ftdi_read_data_set_chunksize(ctx, 0x10000));
	check(ftdi_set_event_char(ctx, 0, 0));
	check(ftdi_set_error_char(ctx, 0, 0));
	check(ftdi_set_latency_timer(ctx, 64));
	check(ftdi_set_bitmode(ctx, 0x0000, BITMODE_MPSSE));
	check(ftdi_usb_reset(ctx));
	check(ftdi_usb_purge_buffers(ctx));
	check(ftdi_setflowctrl(ctx, SIO_RTS_CTS_HS));
	if (ftdi_write_data(ctx, (void *) command, sizeof command) != sizeof command)
		goto l_abort;
	knc_cs_low(ctx);


	// Scan each chip for cores
	int chip_id;
	int devices = 0;
	uint16_t chip_cores[MAX_CHIPS];
	int i;

	for (chip_id = 0; chip_id < MAX_CHIPS; chip_id++) {
		int len;
		chip_cores[chip_id] = 0;
		unsigned char probe_response[2+2];

		struct core_status status;
		int z;
		for (z = 0; z < 10; z++) {
		knc_cs_low(ctx);
		const unsigned char init_probefpgas[] =
		{
		    SET_BITS_LOW, 0x00 | CLOCK_IDLE, 0x0b,
		    SET_BITS_LOW, 0x08 | CLOCK_IDLE, 0x0b,
		    MPSSE_DO_WRITE | WRITE_FLANK, -1 + 1 + 1 + 2, 0, CMD_IDENTIFY, chip_id, 0x00, 0x00
		};
		if (ftdi_write_data(ctx, (void *) init_probefpgas, sizeof init_probefpgas)
		    != sizeof init_probefpgas)
			goto l_abort;
		applog(LOG_DEBUG, "KnC probe %d, sent mpsse %lu bytes: %02x%02x%02x %02x%02x%02x %02x%02x%02x %02x%02x%02x%02x",
		    chip_id,
		    (unsigned long)sizeof init_probefpgas,
		    init_probefpgas[0], init_probefpgas[1], init_probefpgas[2],
		    init_probefpgas[3],
		    init_probefpgas[4], init_probefpgas[5], init_probefpgas[6],
		    init_probefpgas[7], init_probefpgas[8], init_probefpgas[9],

		    init_probefpgas[10], init_probefpgas[11], init_probefpgas[12]);

		const unsigned char read_probe_response[] =
		{
		    MPSSE_DO_READ | READ_FLANK, -1 + sizeof(probe_response), 0,
		    SEND_IMMEDIATE
		};
		if (ftdi_write_data(ctx, (void *) read_probe_response, sizeof read_probe_response)
		    != sizeof read_probe_response)
			goto l_abort;
		applog(LOG_DEBUG, "KnC probe %d, sent mpsse (read %lu byte): %02x%02x%02x%02x",
			    chip_id,
			    (unsigned long)sizeof probe_response, read_probe_response[0], read_probe_response[1], read_probe_response[2], read_probe_response[3]);

		len = ftdi_read_data(ctx, probe_response, sizeof probe_response);
		applog(LOG_DEBUG, "KnC probe %d, read %d byte (wanted %lu): %02x%02x%02x%02x",
			    chip_id,
			    len, (unsigned long)sizeof probe_response,
			    probe_response[0],
			    probe_response[1],
			    probe_response[2],
			    probe_response[3]);
		}
		if (len != sizeof probe_response)
			goto l_abort;
		if ((probe_response[2] << 8 | probe_response[3]) == JUPITER_FIRMWARE_VERSION) {
			int cores = probe_response[0] << 8 | probe_response[1];
			chip_cores[chip_id] = cores;
			devices += cores;
		}
		knc_cs_low(ctx);
	}

	if (!devices) {
		applog(LOG_INFO, "FTDI detected, but not KnCminer core");
		goto l_abort;
	}
	applog(LOG_INFO, "Found a KnC miner with %d cores", devices);

	struct cgpu_info *cgpu = calloc(1, sizeof(*cgpu));
	struct knc_state *knc = calloc(1, sizeof(*knc));
	struct core_state *cores = calloc(devices, sizeof(*cores));
	if (!cgpu || !knc || !cores) {
		applog(LOG_ERR, "KnC miner detected, but failed to allocate memory");
		return false;
	}
	knc->ctx = ctx;
	knc->devices = devices;
	knc->cores = cores;
	int dev = 0;
	for (chip_id = 0; chip_id < MAX_CHIPS; chip_id++) {
		int core_id;
		for (core_id = 0; core_id < chip_cores[chip_id]; core_id++) {
			struct core_status status;
			knc_send_halt(ctx, chip_id, core_id);
			knc_read_status(ctx, chip_id, core_id, &status);
			struct core_state *core = cores + dev++;
			assert(dev <= devices);
			core->prev_progress = 0x0;
			core->active_slot = 0xff;
			core->queued_slot = 0xff;
			core->next_slot = 1;
			core->last_nonce_slot = status.nonce_slot;
			core->last_nonce = status.golden_nonce;
			core->active = NULL;
			core->queued = NULL;
			core->chip_id = chip_id;
			core->core_id = core_id;
		}
	}

	knc_selftest(ctx, cores, devices);

	cgpu->drv = &knc_drv;
	//cgpu->fpgaid=response[1];
	cgpu->name = "KnCminer";
	cgpu->threads = 1;	// .. perhaps our number of cores?

	cgpu->knc_state = knc;
	add_cgpu(cgpu);
	// Icarus driver uses cgpu->device_id to index an info table.
	// That index is set up by add_cgpu()

	knc_cs_low(ctx);
	return true;
      l_abort:
	knc_cs_low(ctx);
	ftdi_free(ctx);
	return false;
}

void
knc_detect(void)
{
	// Probe devices and register with add_cgpu
	//usb_detect(&knc_drv, knc_detect_one);
	int idx;

	for (idx = 0; idx < 32; idx++) {
		/* ORSoC USB JTAG adapter, for DE2 testing */
		struct ftdi_context *ctx = ftdi_new();
		ftdi_set_interface(ctx, INTERFACE_A);
		if (ftdi_usb_open_desc_index(ctx, 0x0403, 0x6010,
			"ORSoC OpenRISC debug cable",
			NULL, idx) < 0) {
			ftdi_free(ctx);
			break;
		}
		knc_detect_one(ctx);
	}
	for (idx = 0; idx < 32; idx++) {
		/* ORSoC USB JTAG adapter, for DE2 testing */
		struct ftdi_context *ctx = ftdi_new();
		ftdi_set_interface(ctx, INTERFACE_B);
		if (ftdi_usb_open_desc_index(ctx, 0x0403, 0x6011,
			NULL,	// Mars prototype board (unprogrammed)
			 NULL, idx) < 0) {
			ftdi_free(ctx);
			break;
		}
		knc_detect_one(ctx);
	}
}

static void
knc_discardactive(struct cgpu_info *cgpu, int dev)
{
	struct knc_state *knc = cgpu->knc_state;
	struct core_state *core = knc->cores + dev;
	if (core->active)
		work_completed(cgpu, core->active);
	core->active = core->queued;
	core->active_slot = core->queued_slot;
	core->queued_slot = 0xff;
	core->queued = NULL;
}


static int64_t
knc_scanwork(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	struct knc_state *knc = cgpu->knc_state;
	bool reports_finished;
	int64_t hashes_estimate = 0;
	const int report_size = 8;
	int device;

	applog(LOG_DEBUG, "KnC running scanwork");

	// return value is number of nonces that have been checked since
	// previous call

	for (device = 0; device < knc->devices; device++) {
		struct core_state *core = knc->cores + device;
		struct core_status status;
		if (knc_read_status(knc->ctx, core->chip_id, core->core_id, &status) != 0)
			goto l_abort;
		uint32_t hashes_delta = (0x100000000LL + status.nonce_checked - core->prev_progress) & 0xffffffff;
		core->prev_progress = status.nonce_checked;
		hashes_estimate += hashes_delta;
		applog(LOG_DEBUG, "KnC speed delta: device(%d:%d) %08x->%08x = %08x",
			device, core->chip_id, core->core_id, core->prev_progress, status.nonce_checked, hashes_delta);
			
		if (status.nonce_slot != core->last_nonce_slot || status.golden_nonce != core->last_nonce) {
			if (status.nonce_slot == core->queued_slot) {
				// Done with this active slot
				knc_discardactive(cgpu, device);
			}
			applog(LOG_INFO, "KnC reported new nonce: %08x %d(%d:%d):%d/%d",
				    status.golden_nonce, device, core->core_id, core->core_id, status.nonce_slot, core->active_slot);
			if (status.nonce_slot == core->active_slot && core->active)
				submit_nonce(thr, core->active, status.golden_nonce);
			else
				applog(LOG_WARNING, "KnC reported nonce not from active slot");
			core->last_nonce_slot = status.nonce_slot;
			core->last_nonce = status.golden_nonce;
		}
		if (status.active_slot == core->queued_slot) {
			// Done with this active slot
			knc_discardactive(cgpu, device);
		}
	}
	
	return hashes_estimate;

      l_abort:
	applog(LOG_ERR, "KnC ftdi error: %s", ftdi_get_error_string(knc->ctx));

	return -1;
}

static bool
knc_queue_full(struct cgpu_info *cgpu)
{
	struct knc_state *knc = cgpu->knc_state;
	int i;
	int queue_full = true;

	applog(LOG_DEBUG, "KnC running queue full");
	for (i = 0; i < knc->devices; i++) {
		if (knc->cores[i].queued == NULL) {
			struct work *work = get_queued(cgpu);
			if (!work) {
				queue_full = false;
				break;
			}
			knc_assign_work(cgpu, i, work);
		}
	}
	// Managed to fill all slots
	applog(LOG_DEBUG, "KnC queue is %s", queue_full ? "full" : "NOT full");
	return queue_full;
}

static void
knc_flush_work(struct cgpu_info *cgpu)
{
	struct knc_state *knc = cgpu->knc_state;
	int dev;

	applog(LOG_ERR, "KnC running flushwork");

	for (dev = 0; dev < knc->devices; dev++) {
		struct core_state *core = knc->cores + dev;
		knc_send_halt(knc->ctx, core->chip_id, core->core_id);
		if (core->active)
			work_completed(cgpu, core->active);
		if (core->queued)
			work_completed(cgpu, core->queued);
		core->active = NULL;
		core->queued = NULL;
	}
}


// Work assignment functions (for use as drv->hash_work)
//  Task: create work, submit to hardware, read nonces, submit to checking
// hash_queued_work is intended to scan full ranges (takes 43s on our device)
//   uses drv->scanwork
// hash_sole_work is intended to work on one attempt at a time, interrupting
// as necessary
//   uses drv->scanhash
// We should probably use hash_sole_work for each FPGA, but how to get 
// reports back?
// Final version likely wants to use queued work, with each FPGA searching
// a subset of nonces, and updating its midstate+data automatically. 

struct device_drv knc_drv =
{
    .drv_id = DRIVER_KNC,
    .dname = "KnCminer",
    .name = "KnC",
    .drv_detect = knc_detect,	// Probe for devices, add with add_cgpu

    .hash_work = hash_queued_work,
    .scanwork = knc_scanwork,
    .queue_full = knc_queue_full,
    .flush_work = knc_flush_work,
};
