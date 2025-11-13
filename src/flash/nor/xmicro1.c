// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2025 Zale Yu                                            *
 *   zale.yu@siliconautotech.com                                           *
 **************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <helper/log.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/target.h>
#include <target/cortex_m.h>
#include <inttypes.h>

#include "../../../contrib/loaders/flash/xmirco1/xmicro1_sram_algo_prog.inc"

static const uint8_t *xmicro1_algo_blob_u8   = xmicro1_sram_algo_prog_bin;
static const size_t   xmicro1_algo_blob_size = xmicro1_sram_algo_prog_bin_len;

/* ==== Device-specific constants ==== */
#define XMICRO1_PFLASH_MEM_BASE          (0x02000000u)
#define XMICRO1_PFLASH_SIZE              (0x00100000u) /* 1MB */
#define XMICRO1_PFLASH_PHY_PAGE_SIZE     (0x00002000u) /* 8KB */

#define XMICRO1_DFLASH_MEM_BASE          (0x20000000u)
#define XMICRO1_DFLASH_SIZE              (0x00010000u) /* 64KB */
#define XMICRO1_DFLASH_PHY_PAGE_SIZE     (0x00000800u) /* 2KB */

#define XMICRO1_PFLASH_NS_MEM_BASE       (0x12000000u)
#define XMICRO1_DFLASH_NS_MEM_BASE       (0x30000000u)

#define XMICRO1_OP_MAX_NUM_OF_BYTES      (16384u)      /* 16KB */
#define XMICRO1_OP_MAX_NUM_OF_WORDS      (XMICRO1_OP_MAX_NUM_OF_BYTES >> 2)

#define XMICRO1_SEC_TO_US                (1000000u)
#define XMICRO1_US_TO_NS                 (1000u)
#define XMICRO1_OP_TIMEOUT_LOOPS         (400u)        /* ~400 ms by alive_sleep(1) */

#define XMICRO1_PFLASH_REG_BASE                      (0x40000000u)
#define XMICRO1_DFLASH_REG_BASE                      (0x40002000u)

/* The following register offsets are shared by both PFLASH and DFLASH controllers */
#define XMICRO1_PFLASH_REG_CONTROL_OFST              (0x20u)
#define XMICRO1_PFLASH_REG_ADDR_OFST                 (0x24u)
#define XMICRO1_PFLASH_REG_DEFAULT_REGION_OFST       (0x90u)
#define XMICRO1_PFLASH_REG_MP_BANK_CFG_SHADOWED_OFST (0x138u)
#define XMICRO1_PFLASH_REG_OP_STATUS_OFST            (0x13Cu)
#define XMICRO1_PFLASH_REG_STATUS_OFST               (0x140u)
#define XMICRO1_PFLASH_REG_ERR_CODE_OFST             (0x148u)
#define XMICRO1_PFLASH_REG_FIFO_RST_OFST             (0x170u)
#define XMICRO1_PFLASH_REG_PROG_FIFO_OFST            (0x178u)
#define XMICRO1_PFLASH_REG_RD_FIFO_OFST              (0x17Cu)
#define XMICRO1_PFLASH_REG_SMW_TIMER_OFST            (0x200u)
#define XMICRO1_PFLASH_REG_TIMER_CYC_OFST            (0x228u)
#define XMICRO1_PFLASH_REG_OP_CTRL_OFST              (0x238u)
#define XMICRO1_PFLASH_REG_EFM_STATUS_OFST           (0x240u)

/* STATUS / OP_STATUS bits:
 * - STATUS[2]: PROG FIFO FULL (1 = full)
 * - STATUS[1]: READ FIFO EMPTY (1 = empty)
 * - STATUS[4]: RECALL IN-PROGRESS? (1 = busy)  -> wait until 0
 * - OP_STATUS[0]: operation busy? (non-zero indicates completion pending/flags)
 * - OP_STATUS[1]: operation error flag (1 = error)
 * - EFM_STATUS[1]: macro busy (1 = busy)
 * - EFM_STATUS[2]: smart write error (1 = error)
 */
#define XMICRO1_STATUS_PROG_FIFO_EMPTY  (1u << 3)
#define XMICRO1_STATUS_PROG_FIFO_FULL   (1u << 2)
#define XMICRO1_STATUS_READ_FIFO_EMPTY  (1u << 1)
#define XMICRO1_STATUS_RECALL_BUSY      (1u << 4)

#define XMICRO1_OP_STATUS_ERR           (1u << 1)

#define XMICRO1_EFM_STATUS_MACRO_BUSY   (1u << 1)
#define XMICRO1_EFM_STATUS_SMW_ERR      (1u << 2)

/* Multi-bit bool encoding */
#define XMICRO1_MBB_TRUE  (0x6u)
#define XMICRO1_MBB_FALSE (0x9u)

/* ==== Private driver state ==== */
struct xmicro1_flash_bank {
	uint32_t reg_base;
	uint32_t page_size;
	uint32_t bank_size;
	uint32_t coreclk_hz;   /* required for timer programming; 0 = unknown (use conservative default) */
    /* --- SRAM algo cache --- */
    struct working_area *algo_wa;
    struct working_area *buf_wa;
    uint32_t             buf_sz;
    bool                 algo_loaded;
};

/* ==== Helpers to read/write target memory ==== */
static inline int xmicro1_wr32(struct target *t, uint32_t addr, uint32_t val)
{
	return target_write_u32(t, addr, val);
}

static inline int xmicro1_rd32(struct target *t, uint32_t addr, uint32_t *val)
{
	return target_read_u32(t, addr, val);
}

/* Wait until lambda(cond_val) becomes true, polling addr every 1 ms, up to loops.
 * cond_fn returns true to stop waiting. */
typedef bool (*xmicro1_cond_fn)(uint32_t v);

static int xmicro1_wait32(struct target *t, uint32_t addr, xmicro1_cond_fn cond, unsigned loops)
{
	int retval;
	uint32_t v = 0;
	for (unsigned i = 0; i < loops; i++) {
		retval = xmicro1_rd32(t, addr, &v);
		if (retval != ERROR_OK)
			return retval;
		if (cond(v))
			return ERROR_OK;
		alive_sleep(1);
	}
	return ERROR_TIMEOUT_REACHED;
}

static bool cond_nonzero(uint32_t v) { return v != 0; }
static bool cond_macro_idle(uint32_t v) { return (v & XMICRO1_EFM_STATUS_MACRO_BUSY) == 0; }
static bool cond_prog_fifo_empty(uint32_t v) { return (v & XMICRO1_STATUS_PROG_FIFO_EMPTY) != 0; }
static bool cond_recall_done(uint32_t v) { return (v & XMICRO1_STATUS_RECALL_BUSY) == 0; }

/* Clear OP_STATUS and ERR_CODE  */
static int xmicro1_clear_status(struct target *t, uint32_t reg_base)
{
	int r;
	uint32_t err_code = 0;
	r = xmicro1_rd32(t, reg_base + XMICRO1_PFLASH_REG_ERR_CODE_OFST, &err_code);
	if (r != ERROR_OK) return r;
	r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_ERR_CODE_OFST, err_code);
	if (r != ERROR_OK) return r;

	/* 'err_code' back into OP_STATUS to clear it; if HW actually requires a different
	 * magic value (e.g. write-1-to-clear), adjust here after bring-up. */
	r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_OP_STATUS_OFST, err_code);
	return r;
}

static int xmicro1_wait_for_idle(struct target *t, uint32_t reg_base)
{
	int r;

	/* 1) Wait for OP_STATUS to become non-zero (operation result latched) */
	r = xmicro1_wait32(t, reg_base + XMICRO1_PFLASH_REG_OP_STATUS_OFST, cond_nonzero, XMICRO1_OP_TIMEOUT_LOOPS);
	if (r != ERROR_OK) {
		LOG_ERROR("XMICRO1: OP_STATUS timeout");
		return r;
	}

	/* 2) Check error flag then clear status */
	uint32_t op_status = 0;
	r = xmicro1_rd32(t, reg_base + XMICRO1_PFLASH_REG_OP_STATUS_OFST, &op_status);
	if (r != ERROR_OK) return r;
	if (op_status & XMICRO1_OP_STATUS_ERR) {
		LOG_ERROR("XMICRO1: operation error, OP_STATUS=0x%08" PRIx32, op_status);
	}
	r = xmicro1_clear_status(t, reg_base);
	if (r != ERROR_OK) return r;

	/* 3) Wait for macro not busy */
	r = xmicro1_wait32(t, reg_base + XMICRO1_PFLASH_REG_EFM_STATUS_OFST, cond_macro_idle, XMICRO1_OP_TIMEOUT_LOOPS);
	if (r != ERROR_OK) {
		LOG_ERROR("XMICRO1: macro busy timeout");
		return r;
	}

	/* 4) Smart-write error bit */
	uint32_t efm = 0;
	r = xmicro1_rd32(t, reg_base + XMICRO1_PFLASH_REG_EFM_STATUS_OFST, &efm);
	if (r != ERROR_OK) return r;
	if (efm & XMICRO1_EFM_STATUS_SMW_ERR) {
		LOG_ERROR("XMICRO1: smart-write error (EFM_STATUS=0x%08" PRIx32 ")", efm);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* Reset FIFOs */
static int xmicro1_fifo_reset(struct target *t, uint32_t reg_base)
{
	int r;
	r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_FIFO_RST_OFST, 0x1);
	if (r != ERROR_OK) return r;
	alive_sleep(1);
	return xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_FIFO_RST_OFST, 0x0);
}

/* Initialize controller registers */
static int xmicro1_hw_init(struct flash_bank *bank)
{
	struct xmicro1_flash_bank *ctx = bank->driver_priv;
	struct target *t = bank->target;
	const uint32_t reg_base = ctx->reg_base;

	uint32_t coreclk = ctx->coreclk_hz;
	if (!coreclk) {
		/* conservative default to avoid div-by-zero; adjust via `XMICRO1 coreclk` */
		coreclk = 24000000u;
		LOG_WARNING("XMICRO1: coreclk_hz not set, using default %u Hz; use `XMICRO1 coreclk <Hz>` to set precisely.",
			coreclk);
	}

	/* SMW_TIMER: [11:0] clk/us, [14:12] clk per 20ns, [24:20] pv_readwt */
	uint32_t num_clk_per_us   = coreclk / XMICRO1_SEC_TO_US;
	uint32_t num_clk_per_ns20 = ((num_clk_per_us * 20u) / XMICRO1_US_TO_NS) + 1u;
	uint32_t pv_readwt        = 20u;

	uint32_t val = ((num_clk_per_us & 0xFFu) << 0)
	             | ((num_clk_per_ns20 & 0x7u) << 12)
	             | ((pv_readwt & 0x1Fu) << 20);
	int r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_SMW_TIMER_OFST, val);
	if (r != ERROR_OK) return r;

	/* TIMER_CYC: use the 'FPGA slower' formula from your code path */
	uint32_t cyc_prd = (((XMICRO1_US_TO_NS / 10u) * 4u) / (num_clk_per_us ? num_clk_per_us : 1u)) & 0xFFu;
	r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_TIMER_CYC_OFST, cyc_prd);
	if (r != ERROR_OK) return r;

	/* default region properties */
	val = ((XMICRO1_MBB_TRUE  & 0xFu) <<  0)  /* rd_en */
	    | ((XMICRO1_MBB_TRUE  & 0xFu) <<  4)  /* prog_en */
	    | ((XMICRO1_MBB_TRUE  & 0xFu) <<  8)  /* erase_en */
	    | ((XMICRO1_MBB_FALSE & 0xFu) << 12)  /* scramble_en */
	    | ((XMICRO1_MBB_TRUE  & 0xFu) << 16)  /* ecc_en */
	    | ((XMICRO1_MBB_FALSE & 0xFu) << 20); /* he_en */
	r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_DEFAULT_REGION_OFST, val);
	if (r != ERROR_OK) return r;

	/* bank erase enable (write twice as in your code) */
	r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_MP_BANK_CFG_SHADOWED_OFST, 0x1);
	if (r != ERROR_OK) return r;
	r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_MP_BANK_CFG_SHADOWED_OFST, 0x1);
	if (r != ERROR_OK) return r;

	/* FIFO reset */
	r = xmicro1_fifo_reset(t, reg_base);
	if (r != ERROR_OK) return r;

	/* Start recall */
	r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_OP_CTRL_OFST, 0x1);
	if (r != ERROR_OK) return r;

	/* Wait recall done: STATUS[4] -> 0 */
	r = xmicro1_wait32(t, reg_base + XMICRO1_PFLASH_REG_STATUS_OFST, cond_recall_done, XMICRO1_OP_TIMEOUT_LOOPS);
	if (r != ERROR_OK) {
		LOG_ERROR("XMICRO1: recall timeout");
		return r;
	}

	return ERROR_OK;
}

/* ==== Flash driver ops ==== */

static int xmicro1_erase(struct flash_bank *bank, unsigned first, unsigned last)
{
	struct xmicro1_flash_bank *ctx = bank->driver_priv;
	struct target *t = bank->target;
	const uint32_t reg_base = ctx->reg_base;
	int r;

	r = xmicro1_hw_init(bank);
	if (r != ERROR_OK)
		return r;

	/* Whole-bank erase optimization if caller asks full range */
	bool whole_bank = (first == 0) && (last + 1 == bank->num_sectors);

	if (whole_bank) {
		/* ADDRESS = 0; CONTROL: bank erase */
		r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_ADDR_OFST, 0x0);
		if (r != ERROR_OK) return r;

		/* CONTROL fields: op_en=1, op=2(erase), ERASE_SEL=1 */
		uint32_t ctrl = ((1u & 0x1u) << 0) | ((2u & 0x3u) << 4) | ((1u & 0x1u) << 7) | ((0u & 0x1u) << 8);
		r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_CONTROL_OFST, ctrl);
		if (r != ERROR_OK) return r;

		r = xmicro1_wait_for_idle(t, reg_base);
		if (r != ERROR_OK) return r;
	} else {
		/* Page erase in chunks */
		for (unsigned s = first; s <= last; ) {
			unsigned run = 1;
			/* merge consecutive sectors into one multi-page erase (controller supports num_of_pages-1 in CONTROL[27:16]) */
			while ((s + run) <= last && run < 0x1000u)
				run++;

			uint32_t off = bank->sectors[s].offset; /* offset relative to bank base */
			/* FIFO reset before operation  */
			r = xmicro1_fifo_reset(t, reg_base);
			if (r != ERROR_OK) return r;

			/* ADDRESS = offset */
			r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_ADDR_OFST, off);
			if (r != ERROR_OK) return r;

			/* CONTROL: op_en=1, op=2(erase), ERASE_SEL=0, num=(run-1) */
			uint32_t ctrl = ((1u & 0x1u) << 0)
			              | ((2u & 0x3u) << 4)
			              | ((0u & 0x1u) << 7)
			              | ((0u & 0x1u) << 8)
			              | (((run - 1u) & 0xFFFu) << 16);
			r = xmicro1_wr32(t, reg_base + XMICRO1_PFLASH_REG_CONTROL_OFST, ctrl);
			if (r != ERROR_OK) return r;

			r = xmicro1_wait_for_idle(t, reg_base);
			if (r != ERROR_OK) return r;

			s += run;
		}
	}

	return ERROR_OK;
}

static int xmicro1_write(struct flash_bank *bank, const uint8_t *buffer,
                         uint32_t offset, uint32_t count)
{
    struct xmicro1_flash_bank *ctx = bank->driver_priv;
    struct target *t = bank->target;
    const uint32_t reg_base = ctx->reg_base;
    int r;

    /* --- Initialize controller and sanity checks --- */
    r = xmicro1_hw_init(bank);
    if (r != ERROR_OK)
        return r;

    if ((count == 0) || ((offset + count) > bank->size) || (offset & 0x3)) {
        LOG_ERROR("XMICRO1: invalid write args (off=0x%08" PRIx32 ", count=%" PRIu32 ")",
                  offset, count);
        return ERROR_FAIL;
    }

    /* --- ensure algo working areas --- */
    if (!ctx->algo_loaded) {
        if (!ctx->algo_wa) {
            r = target_alloc_working_area(t, xmicro1_algo_blob_size, &ctx->algo_wa);
            if (r != ERROR_OK) {
                LOG_ERROR("XMICRO1: alloc algo wa failed");
                return r;
            }
        }
        r = target_write_buffer(t, ctx->algo_wa->address,
                                xmicro1_algo_blob_size, xmicro1_algo_blob_u8);
        if (r != ERROR_OK) {
            LOG_ERROR("XMICRO1: write algo blob failed");
            return r;
        }
        ctx->algo_loaded = true;
    }

    /* allocate buffer working area（max 32KB and should not be over 16KB） */
    uint32_t desired = 32u * 1024u;
    if (desired > XMICRO1_OP_MAX_NUM_OF_BYTES)
        desired = XMICRO1_OP_MAX_NUM_OF_BYTES;

    if (!ctx->buf_wa || ctx->buf_sz < desired) {
        if (ctx->buf_wa)
            target_free_working_area(t, ctx->buf_wa);
        ctx->buf_wa = NULL;
        ctx->buf_sz = 0;

        r = target_alloc_working_area(t, desired, &ctx->buf_wa);
        if (r != ERROR_OK) {
            /* try to use 8KB */
            desired = 8u * 1024u;
            if (desired > XMICRO1_OP_MAX_NUM_OF_BYTES)
                desired = XMICRO1_OP_MAX_NUM_OF_BYTES;
            r = target_alloc_working_area(t, desired, &ctx->buf_wa);
            if (r != ERROR_OK) {
                LOG_ERROR("XMICRO1: alloc data wa failed");
                return r;
            }
        }
        ctx->buf_sz = desired;
    }

    uint32_t written = 0;
    long long t_start = timeval_ms();

    LOG_INFO("XMICRO1: SRAM algo write off=0x%08" PRIx32 ", size=%" PRIu32 ", buf=%u",
             offset, count, ctx->buf_sz);

    while (written < count) {
        uint32_t chunk = MIN(count - written, ctx->buf_sz);
        chunk &= ~0x3u;
        if (chunk == 0) break;

        r = target_write_buffer(t, ctx->buf_wa->address, chunk, &buffer[written]);
        if (r != ERROR_OK) {
            LOG_ERROR("XMICRO1: write buf to wa failed");
            return r;
        }

        /* prepare Thumb arguments */
        struct reg_param regs[4];
        struct armv7m_algorithm arm_info;
        memset(&arm_info, 0, sizeof(arm_info));
        arm_info.common_magic = ARMV7M_COMMON_MAGIC;
        arm_info.core_mode = ARM_MODE_THREAD;

        init_reg_param(&regs[0], "r0", 32, PARAM_IN_OUT);
        init_reg_param(&regs[1], "r1", 32, PARAM_OUT);
        init_reg_param(&regs[2], "r2", 32, PARAM_OUT);
        init_reg_param(&regs[3], "r3", 32, PARAM_OUT);

        buf_set_u32(regs[0].value, 0, 32, reg_base);
        buf_set_u32(regs[1].value, 0, 32, offset + written);
        buf_set_u32(regs[2].value, 0, 32, ctx->buf_wa->address);
        buf_set_u32(regs[3].value, 0, 32, (chunk >> 2)); /* words */

        /* Timeout: Rough estimate is 16 KB / (tens of microseconds per word)
					set it larger, for example around 10 seconds. */
        r = target_run_algorithm(t,
                                 0, NULL,                   /* no mem param */
                                 4, regs,
                                 ctx->algo_wa->address | 1, /* Thumb entry */
                                 0, 10000,                  /* 10s */
                                 &arm_info);

        int retcode = 0;
        if (r == ERROR_OK) {
            /* r0 as the algorithm’s return code. */
            retcode = buf_get_u32(regs[0].value, 0, 32);
        }

        for (int i = 0; i < 4; i++) destroy_reg_param(&regs[i]);

        if (r != ERROR_OK) {
            LOG_ERROR("XMICRO1: target_run_algorithm failed (%d)", r);
            return r;
        }
        if (retcode != 0) {
            LOG_ERROR("XMICRO1: algo returned %d", retcode);
            return ERROR_FAIL;
        }

        written += chunk;

        unsigned percent = (unsigned)(((uint64_t)written * 100u) / count);
        LOG_INFO("XMICRO1: written %u%% (%u/%u bytes)", percent, written, count);
    }

    long long t_end = timeval_ms();
    LOG_INFO("XMICRO1: write done via SRAM algo, total=%u bytes, elapsed=%lld ms",
             written, (t_end - t_start));

    return (written == count) ? ERROR_OK : ERROR_FAIL;
}

static int xmicro1_probe(struct flash_bank *bank)
{
	struct xmicro1_flash_bank *ctx = bank->driver_priv;

	if (!ctx->page_size || !bank->size)
		return ERROR_FAIL;

	unsigned sectors = bank->size / ctx->page_size;
	if (sectors == 0)
		return ERROR_FAIL;

	free(bank->sectors);
	bank->sectors = calloc(sectors, sizeof(struct flash_sector));
	if (!bank->sectors)
		return ERROR_FAIL;

	bank->num_sectors = sectors;
	for (unsigned i = 0; i < sectors; i++) {
		bank->sectors[i].offset = i * ctx->page_size;
		bank->sectors[i].size   = ctx->page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	ctx->bank_size = bank->size;
	return ERROR_OK;
}

static int xmicro1_auto_probe(struct flash_bank *bank)
{
	if (bank->num_sectors > 0)
		return ERROR_OK;
	return xmicro1_probe(bank);
}

/* We don't implement protect/read/erase_check; use defaults. */
FLASH_BANK_COMMAND_HANDLER(xmicro1_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct xmicro1_flash_bank *ctx = calloc(1, sizeof(*ctx));
	if (!ctx)
		return ERROR_FAIL;

	bank->driver_priv = ctx;

	/* Decide controller macro (reg_base) and physical page size by bank->base.
	 * PFLASH: 8KB page; DFLASH: 2KB page.
	 * Secure/Non-secure windows share the same controller registers. */
	if (bank->base == XMICRO1_PFLASH_MEM_BASE || bank->base == XMICRO1_PFLASH_NS_MEM_BASE) {
		ctx->reg_base  = XMICRO1_PFLASH_REG_BASE;
		ctx->page_size = XMICRO1_PFLASH_PHY_PAGE_SIZE;
	} else if (bank->base == XMICRO1_DFLASH_MEM_BASE || bank->base == XMICRO1_DFLASH_NS_MEM_BASE) {
		ctx->reg_base  = XMICRO1_DFLASH_REG_BASE;
		ctx->page_size = XMICRO1_DFLASH_PHY_PAGE_SIZE;
	} else {
		/* Fallback to PFLASH defaults, but warn so users notice. */
		ctx->reg_base  = XMICRO1_PFLASH_REG_BASE;
		ctx->page_size = XMICRO1_PFLASH_PHY_PAGE_SIZE;
		LOG_WARNING("XMICRO1: unknown bank base 0x%08" PRIx32 ", defaulting to PFLASH params.", (uint32_t)bank->base);
	}

	ctx->bank_size  = bank->size;
	ctx->coreclk_hz = 0; /* let user set or fall back to default */

	/* Optional extra arg: coreclk_hz */
	if (CMD_ARGC >= 7) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], ctx->coreclk_hz);
	}

	return ERROR_OK;
}

static int xmicro1_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct xmicro1_flash_bank *ctx = bank->driver_priv;

	const char *which =
		(bank->base == XMICRO1_PFLASH_MEM_BASE)    ? "PFLASH_S" :
		(bank->base == XMICRO1_PFLASH_NS_MEM_BASE) ? "PFLASH_NS" :
		(bank->base == XMICRO1_DFLASH_MEM_BASE)    ? "DFLASH_S" :
		(bank->base == XMICRO1_DFLASH_NS_MEM_BASE) ? "DFLASH_NS" : "UNKNOWN";

	command_print_sameline(cmd,
		"XMICRO1 %s: base=0x%08" PRIx32 ", size=%u KB, page=%u B, reg_base=0x%08" PRIx32,
		which, (uint32_t)bank->base, (unsigned)(bank->size / 1024), (unsigned)(ctx->page_size), ctx->reg_base);
	if (ctx->coreclk_hz)
		command_print(cmd, ", coreclk=%u Hz", (unsigned)ctx->coreclk_hz);
	else
		command_print(cmd, ", coreclk=<unset>");
	return ERROR_OK;
}

static void xmicro1_free_driver_priv(struct flash_bank *bank)
{
	if (!bank || !bank->driver_priv)
		return;

	struct xmicro1_flash_bank *ctx = bank->driver_priv;
	struct target *t = bank->target;

	/* Free SRAM working areas on the target side, if they were allocated */
	if (ctx->algo_wa) {
		target_free_working_area(t, ctx->algo_wa);
		ctx->algo_wa = NULL;
	}

	if (ctx->buf_wa) {
		target_free_working_area(t, ctx->buf_wa);
		ctx->buf_wa = NULL;
		ctx->buf_sz = 0;
	}

	/* Finally free driver_priv itself */
	default_flash_free_driver_priv(bank);
}

/* ==== Extra user command: 'XMICRO1 coreclk <Hz>' ==== */

COMMAND_HANDLER(xmicro1_handle_coreclk_cmd)
{
	struct flash_bank *bank;
	struct xmicro1_flash_bank *ctx;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	ctx = bank->driver_priv;
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t hz = 0;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], hz);
	ctx->coreclk_hz = hz;
	LOG_INFO("XMICRO1: set coreclk_hz=%u", (unsigned)hz);

	return ERROR_OK;
}

static const struct command_registration xmicro1_command_handlers[] = {
	{
		.name = "coreclk",
		.handler = xmicro1_handle_coreclk_cmd,
		.mode = COMMAND_EXEC,
		.usage = "<Hz>",
		.help = "Set core clock (Hz) for XMICRO1 flash controller timing.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration xmicro1_bank_command_handlers[] = {
	{
		.name = "XMICRO1",
		.mode = COMMAND_ANY,
		.help = "XMICRO1 flash driver commands",
		.usage = "",
		.chain = xmicro1_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver xmicro1_flash = {
	.name = "xmicro1",
	.commands = xmicro1_bank_command_handlers,

	.flash_bank_command = xmicro1_flash_bank_command,
	.erase = xmicro1_erase,
	.write = xmicro1_write,
	.read = default_flash_read,
	.probe = xmicro1_probe,
	.auto_probe = xmicro1_auto_probe,
	.erase_check = default_flash_blank_check,
	.info = xmicro1_info,
	.free_driver_priv = xmicro1_free_driver_priv,
};
