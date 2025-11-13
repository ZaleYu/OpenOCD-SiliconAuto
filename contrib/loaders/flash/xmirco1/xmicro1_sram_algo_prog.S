/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2025 Zale Yu                                            *
 *   zale.yu@siliconautotech.com                                           *
 ***************************************************************************/
    .syntax unified
    .thumb

    .thumb_func
    .global xmicro1_algo_prog_chunk

/* ---------------------------------------------------------------------
 * XMICRO1 flash controller register offsets
 * These must match the C driver definitions.
 * --------------------------------------------------------------------- */
.equ XMICRO1_PFLASH_REG_CONTROL_OFST,              0x20
.equ XMICRO1_PFLASH_REG_ADDR_OFST,                 0x24
.equ XMICRO1_PFLASH_REG_OP_STATUS_OFST,            0x13C
.equ XMICRO1_PFLASH_REG_STATUS_OFST,               0x140
.equ XMICRO1_PFLASH_REG_ERR_CODE_OFST,             0x148
.equ XMICRO1_PFLASH_REG_FIFO_RST_OFST,             0x170
.equ XMICRO1_PFLASH_REG_PROG_FIFO_OFST,            0x178
.equ XMICRO1_PFLASH_REG_EFM_STATUS_OFST,           0x240

/* STATUS bits */
.equ XMICRO1_STATUS_PROG_FIFO_EMPTY_MASK,          (1 << 3)

/* OP_STATUS bits */
.equ XMICRO1_OP_STATUS_ERR_MASK,                   (1 << 1)

/* EFM_STATUS bits */
.equ XMICRO1_EFM_STATUS_MACRO_BUSY_MASK,           (1 << 1)
.equ XMICRO1_EFM_STATUS_SMW_ERR_MASK,              (1 << 2)

/* Timeout loop count for polling. */
.equ TIMEOUT_BULK,                                 500000

/* Helper macro: load TIMEOUT_BULK into a 32-bit register */
    .macro LOAD_TIMEOUT reg
        movw \reg, #:lower16:TIMEOUT_BULK
        movt \reg, #:upper16:TIMEOUT_BULK
    .endm

/* ---------------------------------------------------------------------
 * uint32_t xmicro1_algo_prog_chunk(
 *     uint32_t reg_base,   // R0
 *     uint32_t dst_offset, // R1
 *     uint32_t src_ptr,    // R2
 *     uint32_t words       // R3 (number of 32-bit words)
 * );
 *
 * Return codes in R0:
 *   0 = success
 *   1 = OP_STATUS error bit set
 *   2 = EFM_STATUS smart-write error bit set
 *   3 = timeout in polling loops
 *
 * Clobbers: R4-R7, APSR flags, LR
 * Ends execution with BKPT so that OpenOCD can detect completion.
 * --------------------------------------------------------------------- */
xmicro1_algo_prog_chunk:
    push {r4-r7, lr}

    /* Optionally disable interrupts while programming:
     * (uncomment if needed, and store/restore PRIMASK in r7)
     *
     *   mrs   r7, PRIMASK
     *   cpsid i
     */

    /* -----------------------------------------------------------------
     * FIFO reset: write 1 then 0 to FIFO_RST register
     * ----------------------------------------------------------------- */
    movs    r4, #1
    str     r4, [r0, #XMICRO1_PFLASH_REG_FIFO_RST_OFST]
    movs    r4, #0
    str     r4, [r0, #XMICRO1_PFLASH_REG_FIFO_RST_OFST]

    /* -----------------------------------------------------------------
     * Program address:
     *   ADDR = dst_offset (relative to the bank base)
     * ----------------------------------------------------------------- */
    str     r1, [r0, #XMICRO1_PFLASH_REG_ADDR_OFST]

    /* -----------------------------------------------------------------
     * CONTROL register setup:
     *   NUM   = (words - 1) in bits [27:16]
     *   OP    = 1 (program) in bits [5:4]
     *   START = 1 in bit [0]
     * ----------------------------------------------------------------- */
    subs    r4, r3, #1          /* r4 = words - 1 */
    lsls    r4, r4, #16         /* shift into NUM field */
    movs    r5, #1
    orrs    r4, r5              /* START = 1 */
    movs    r5, #(1 << 4)
    orrs    r4, r5              /* OP = 1 (program) */
    str     r4, [r0, #XMICRO1_PFLASH_REG_CONTROL_OFST]

    /* -----------------------------------------------------------------
     * Send data into PROG_FIFO in bursts of up to 16 words.
     * r3 = remaining word count
     * r2 = src_ptr (advances by 4 bytes per word)
     * r6 = constant maximum burst size (16)
     * r7 = current burst size
     * r5 = inner loop counter
     * ----------------------------------------------------------------- */
    movs    r6, #16             /* maximum burst size */

send_loop:
    cmp     r3, #0
    beq     prog_wait           /* no words left -> go wait for completion */

    /* r7 = burst size = min(r3, 16) */
    mov     r7, r3
    cmp     r7, r6
    bls     burst_size_ok
    mov     r7, r6
burst_size_ok:

    /* r5 = burst loop counter */
    mov     r5, r7

burst_loop:
    ldr     r4, [r2], #4        /* load one word from src_ptr */
    str     r4, [r0, #XMICRO1_PFLASH_REG_PROG_FIFO_OFST]
    subs    r5, r5, #1
    bne     burst_loop

    /* remaining words: r3 -= burst_size (r7) */
    subs    r3, r3, r7

    /* If there are still words remaining, wait for PROG_FIFO_EMPTY
     * before sending the next burst.
     */
    cmp     r3, #0
    beq     prog_wait

    /* -----------------------------------------------------------------
     * Poll STATUS.PROG_FIFO_EMPTY until it becomes 1 or timeout.
     * ----------------------------------------------------------------- */
    LOAD_TIMEOUT r5

fifo_poll:
    ldr     r4, [r0, #XMICRO1_PFLASH_REG_STATUS_OFST]
    tst     r4, #XMICRO1_STATUS_PROG_FIFO_EMPTY_MASK
    bne     send_loop           /* empty -> FIFO ready for next burst */
    subs    r5, r5, #1
    bne     fifo_poll

    /* Timeout while waiting for PROG_FIFO_EMPTY */
    movs    r0, #3
    b       exit

    /* -----------------------------------------------------------------
     * After all data words have been pushed into FIFO:
     *   Wait for OP_STATUS != 0.
     * ----------------------------------------------------------------- */
prog_wait:
    LOAD_TIMEOUT r5

op_poll:
    ldr     r4, [r0, #XMICRO1_PFLASH_REG_OP_STATUS_OFST]
    cmp     r4, #0
    bne     op_have_status
    subs    r5, r5, #1
    bne     op_poll

    /* Timeout while waiting for OP_STATUS result */
    movs    r0, #3
    b       exit

op_have_status:
    /* Check OP_STATUS error bit: if set, return 1 */
    tst     r4, #XMICRO1_OP_STATUS_ERR_MASK
    beq     clear_status
    movs    r0, #1
    b       exit

    /* -----------------------------------------------------------------
     * Clear error/status bits:
     *   - read ERR_CODE
     *   - write it back to ERR_CODE and OP_STATUS (W1C behavior)
     * ----------------------------------------------------------------- */
clear_status:
    ldr     r5, [r0, #XMICRO1_PFLASH_REG_ERR_CODE_OFST]
    str     r5, [r0, #XMICRO1_PFLASH_REG_ERR_CODE_OFST]
    str     r5, [r0, #XMICRO1_PFLASH_REG_OP_STATUS_OFST]

    /* -----------------------------------------------------------------
     * Wait until EFM_STATUS.MACRO_BUSY == 0.
     * ----------------------------------------------------------------- */
    LOAD_TIMEOUT r5

busy_poll:
    ldr     r4, [r0, #XMICRO1_PFLASH_REG_EFM_STATUS_OFST]
    tst     r4, #XMICRO1_EFM_STATUS_MACRO_BUSY_MASK
    beq     check_smw_error
    subs    r5, r5, #1
    bne     busy_poll

    /* Timeout while waiting for macro to become idle */
    movs    r0, #3
    b       exit

    /* -----------------------------------------------------------------
     * Check smart-write error bit in EFM_STATUS.
     * If set, return 2. Otherwise, success.
     * ----------------------------------------------------------------- */
check_smw_error:
    tst     r4, #XMICRO1_EFM_STATUS_SMW_ERR_MASK
    beq     prog_ok
    movs    r0, #2
    b       exit

prog_ok:
    movs    r0, #0

    /* -----------------------------------------------------------------
     * Common exit path:
     *   - Optionally restore PRIMASK if interrupts were disabled.
     *   - Restore callee-saved registers.
     *   - Trigger BKPT so that OpenOCD sees the target halted.
     * ----------------------------------------------------------------- */
exit:
    /* If you used PRIMASK save/restore:
     *   msr PRIMASK, r7
     */

    pop     {r4-r7, lr}
    bkpt    0xab          /* signal completion to debugger */
    b       .             /* safety loop if continued by mistake */
