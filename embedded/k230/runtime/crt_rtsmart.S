/*
 * Minimal CRT for RT-Smart user processes on K230 (RISC-V 64).
 *
 * RT-Smart passes arguments differently from Linux:
 *   a0 = pointer to argc/argv/envp structure
 *   a2 = stack pointer
 *
 * Standard Linux musl CRT assumes sp already points to argc/argv/envp.
 * This stub bridges the gap: set sp from a2, then fall through to
 * _start_c which calls __libc_start_main.
 */

.section .text
.global _start
.type _start, @function

_start:
    /* RT-Smart kernel passes the stack address in a2. */
    mv      sp, a2

    /* Clear the frame pointer for clean backtraces. */
    li      s0, 0

    /* Align the stack to 16 bytes (RISC-V ABI requirement). */
    and     sp, sp, -16

    /* _start_c(a0) — a0 already has the argc/argv/envp pointer. */
    j       _start_c
