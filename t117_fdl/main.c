#include "common.h"
#include "cmd_def.h"
#include "packet.h"
#include "channel.h"

#if WITH_EFUSE
#include "efuse.h"
#endif

static struct {
	uintptr_t start;
	uint32_t size, recv;
} dl_status;

static int data_start(uint8_t *pkt) {
	unsigned len = READ16_BE(pkt + 2);
	uint32_t addr = READ32_BE(pkt + 4);
	uint32_t size = READ32_BE(pkt + 8);

	if (len != 8)
		return BSL_REP_INVALID_CMD;

	dl_status.start = addr;
	dl_status.size = size;
	dl_status.recv = 0;
#if !FDL2 && SDRAM_INIT == 2
	// init RAM on demand
	if (addr - 0x80000000 < 64 << 20) {
		static int done = 0;
		if (!done && sdram_init()) {
			dl_send_ack(BSL_REP_OPERATION_FAILED);
			for (;;);
		}
		done = 1;
	}
#endif
	return BSL_REP_ACK;
}

static int data_midst(uint8_t *pkt) {
	unsigned len = READ16_BE(pkt + 2);
	unsigned remain = dl_status.size - dl_status.recv;
	uint32_t addr;
	uint8_t *src = pkt + 4;

	if (len > remain)
		return BSL_REP_DOWN_SIZE_ERROR;

	addr = dl_status.start + dl_status.recv;
	dl_status.recv += len;
	memcpy((uint8_t*)addr, src, len);
	return BSL_REP_ACK;
}

static int data_end(void) {
	uint32_t recv = dl_status.recv;
	uint32_t size = dl_status.size;

	if (size != recv)
		return BSL_REP_DOWN_EARLY_END;
	return BSL_REP_ACK;
}

static int data_exec(void) {
	typedef void (*entry_t)(void);
	uintptr_t start = dl_status.start;

	if (start == ~0u)
		return BSL_REP_INVALID_CMD;

	start += 0x200; // skip DHTB header
	((entry_t)start)();
	return BSL_REP_ACK;
}

static int set_baudrate(uint8_t *pkt) {
	unsigned len = READ16_BE(pkt + 2);
	uint32_t baudrate;

	if (len != 4)
		return BSL_REP_INVALID_CMD;

	baudrate = READ32_BE(pkt + 4);
	dl_channel->open(dl_channel, baudrate);
	return BSL_REP_ACK;
}

#define SFC_BASE 0x20300000

static void sfc_setup(void) {
	MEM4(0x20e00000) |= 0xFFFFFFFFu;
	MEM4(0x21500068) = (MEM4(0x21500068) & ~7u) | 1u;

	MEM4(SFC_BASE + 0x00) = 1;
	MEM4(SFC_BASE + 0x08) = 1;
	MEM4(SFC_BASE + 0x0C) = 0xFF;
}

static void sfc_exec(void) {
	MEM4(SFC_BASE + 0x0C) = 0xFF;
	MEM4(SFC_BASE + 0x04) = 1;
	for (volatile int i = 0; i < 4096; i++) {
		if (MEM4(SFC_BASE + 0x10) & 1) break;
	}
	MEM4(SFC_BASE + 0x0C) = 0xFF;
}

static uint32_t sfc_jedec_id(void) {
	MEM4(SFC_BASE + 0x08) = 1;
	MEM4(SFC_BASE + 0x2C) = (3 << 16) | 0x9F;
	MEM4(SFC_BASE + 0x30) = 0;
	MEM4(SFC_BASE + 0x34) = (1 << 0) | (1 << 2);
	MEM4(SFC_BASE + 0x38) = 0;
	sfc_exec();
	return MEM4(SFC_BASE + 0x58) & 0xFFFFFF;
}

static void sfc_read_bytes(uint32_t addr, uint8_t *dst, uint32_t len) {
	while (len > 0) {
		uint32_t chunk = len > 8 ? 8 : len;
		uint32_t w0, w1;
		MEM4(SFC_BASE + 0x08) = 1;
		MEM4(SFC_BASE + 0x2C) = (chunk << 16) | 0x03;
		MEM4(SFC_BASE + 0x30) = addr & 0xFFFFFF;
		MEM4(SFC_BASE + 0x34) = (1 << 0) | (1 << 1) | (1 << 2);
		MEM4(SFC_BASE + 0x38) = 0;
		sfc_exec();
		w0 = MEM4(SFC_BASE + 0x58);
		w1 = MEM4(SFC_BASE + 0x5C);
		memcpy(dst, &w0, chunk > 4 ? 4 : chunk);
		if (chunk > 4) memcpy(dst + 4, &w1, chunk - 4);
		addr += chunk;
		dst += chunk;
		len -= chunk;
	}
}

static int read_flash(uint8_t *pkt) {
	unsigned len = READ16_BE(pkt + 2);
	uint32_t addr = READ32_BE(pkt + 4);
	uint32_t size = READ32_BE(pkt + 8);
	uint32_t offs = -1;

	if (len == 12) offs = READ32_BE(pkt + 12);
	if (offs != ~0u) {
		addr += offs;
	}

	pkt = dl_send_buf();
	WRITE16_BE(pkt, BSL_REP_READ_FLASH);
	WRITE16_BE(pkt + 2, size);
	if (addr >= 0xffff0000) {
		memcpy(pkt + 4, (void*)addr, size);
	} else if (addr == 0x7F000000) {
		uint32_t *out = (uint32_t*)(pkt + 4);
		memset(out, 0, size);
		*out++ = 0x11111111;
		sfc_setup();
		*out++ = sfc_jedec_id();
		*out++ = MEM4(SFC_BASE + 0x58);
		*out++ = MEM4(SFC_BASE + 0x5C);
		sfc_read_bytes(0, (uint8_t*)(pkt + 32), 32);
		*out++ = 0x66666666;
	} else {
		static int sfc_done = 0;
		if (!sfc_done) {
			sfc_setup();
			sfc_done = 1;
		}
		sfc_read_bytes(addr, pkt + 4, size);
	}
	dl_send_packet(pkt);
	return 0;
}

#if WITH_EFUSE
static int read_efuse(uint8_t *pkt) {
	unsigned len = READ16_BE(pkt + 2);
	unsigned i, n;

	if (len) return BSL_REP_INVALID_CMD;

	pkt = dl_send_buf();
	WRITE16_BE(pkt, BSL_REP_READ_FLASH);

	efuse_init();
	n = EFUSE_MAX;
	for (i = 0; i < n; i++)
		((uint32_t*)(pkt + 4))[i] = efuse_read(i, 0);
	efuse_off();

	WRITE16_BE(pkt + 2, n * 4);
	dl_send_packet(pkt);
	return 0;
}
#endif

void dl_main(void) {
	uint8_t *pkt;
#if 0
	static const char version[] = { "Spreadtrum Boot Block version 1.1" };
#else
#if FDL2
#define FDL_NAME "FDL2"
#else
#define FDL_NAME "FDL1"
#endif
	static char version[] = { "Custom " FDL_NAME ": CHIP ID = 0x00000000" };
	{
		uint32_t i = 25, t, a;
		for (t = chip_id; t; t <<= 4) {
			a = t >> 28;
			if (a >= 10) a += 'a' - '0' - 10;
			version[i++] = a + '0';
		}
	}
#endif

	dl_packet_init();
	dl_status.start = ~0;

#if FDL2
	(void)version;
	dl_send_ack(BSL_REP_ACK);
#else
	for (;;) {
		int ch = dl_channel->getchar(dl_channel, 1);
		if (ch == HDLC_HEADER) break;
	}

	pkt = dl_send_buf();
	WRITE16_BE(pkt, BSL_REP_VER);
	WRITE16_BE(pkt + 2, sizeof(version));
	memcpy(pkt + 4, version, sizeof(version));
	dl_send_packet(pkt);
#endif

	for (;;) {
		pkt = dl_get_packet();
		int type = READ16_BE(pkt);
		int ret;
		switch (type) {
		case BSL_CMD_CONNECT:
			ret = BSL_REP_ACK;
			break;

		case BSL_CMD_START_DATA:
			ret = data_start(pkt);
			break;

		case BSL_CMD_MIDST_DATA:
			ret = data_midst(pkt);
			break;

		case BSL_CMD_END_DATA:
			ret = data_end();
			break;

		case BSL_CMD_EXEC_DATA:
			ret = data_exec();
			break;

		case BSL_CMD_CHANGE_BAUD:
			ret = set_baudrate(pkt);
			break;

		case BSL_CMD_READ_FLASH:
			ret = read_flash(pkt);
			break;

		case BSL_CMD_OFF_CHG:
			ret = BSL_REP_ACK;
			break;

#if WITH_EFUSE
		case 0x6566: // "ef"
			ret = read_efuse(pkt);
			break;
#endif

		default:
			ret = BSL_REP_UNKNOWN_CMD;
		}
		if (ret > 0)
			dl_send_ack(ret);
	}
}
