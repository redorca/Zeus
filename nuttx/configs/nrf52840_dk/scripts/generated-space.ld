
MEMORY
{


    kflash (rx) : ORIGIN = 
                               0
                                                  , LENGTH = 256K
    uflash (rx) : ORIGIN = 
                               0x00040000
                                                     , LENGTH = 256K
    xflash (rx) : ORIGIN = 0x00080000, LENGTH = 512K



    ksram (rwx) : ORIGIN = 
                               0x20000000
                                               , LENGTH = 16K
    usram (rwx) : ORIGIN = 
                               0x20000000 
                                                + LENGTH(ksram), LENGTH = 8K
    xsram (rwx) : ORIGIN = ORIGIN(usram) + LENGTH(usram), LENGTH = 232K
}

ASSERT(ORIGIN(uflash) >= (ORIGIN(kflash) + LENGTH(kflash)), "user flash is OVERLAY with kernel flash");

OUTPUT_ARCH(arm)
SECTIONS
{
    .userspace : {
        *(.userspace)
    } > uflash

    .text : {
        _stext = ABSOLUTE(.);
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > uflash

    .init_section : {
        _sinit = ABSOLUTE(.);
        *(.init_array .init_array.*)
        _einit = ABSOLUTE(.);
    } > uflash

    .ARM.extab : {
        *(.ARM.extab*)
    } > uflash

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > uflash

    __exidx_end = ABSOLUTE(.);

    _eronly = ABSOLUTE(.);

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        _edata = ABSOLUTE(.);
    } > usram AT > uflash

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        _ebss = ABSOLUTE(.);
    } > usram



    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
