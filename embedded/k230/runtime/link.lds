/*
 * RT-Smart user application linker script for RISC-V 64-bit.
 *
 * The RT-Smart kernel validates that the ELF entry point is exactly
 * 0x200000000. Any other address causes "elf entry is invalidate".
 *
 * Source: K230 SDK src/big/rt-smart/userapps/linker_scripts/riscv64/link.lds
 * License: Apache-2.0 (RT-Thread Development Team)
 */

OUTPUT_ARCH( "riscv" )

ENTRY(_start)
SECTIONS
{
    . = 0x200000000;

    .text :
    {
        __text_start__ = .;
        *(.start);
        *(.text*)

        KEEP(*(.init))
        KEEP(*(.fini))

        *crtbegin.o(.ctors)
        *crtbegin?.o(.ctors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
        *(SORT(.ctors.*))
        *(.ctors)

        *crtbegin.o(.dtors)
        *crtbegin?.o(.dtors)
        *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
        *(SORT(.dtors.*))
        *(.dtors)

        *(.rodata*)

        KEEP(*(.eh_frame*))

        . = ALIGN(8);
        _etext = .;
        __text_end__ = .;
    }

    . = ALIGN(0x1000);

    .plt :
    {
        *(*.plt)
    }
    .got :
    {
        __got_start__ = .;
        *(*.got)
        __got_end__ = .;
    }

    data :
    {
        __data_start__ = .;

        *(vtable)
        *(.data*)

        . = ALIGN(8);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP(*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);

        . = ALIGN(8);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);

        . = ALIGN(8);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array))
        PROVIDE_HIDDEN (__fini_array_end = .);

        . = ALIGN(8);
        __data_end__ = .;
    }

    bss :
    {
        __bss_start__ = .;
        *(.bss*)
        *(COMMON)
        *(.stack)
        __bss_end__ = .;
    }

    _end = .;
}
