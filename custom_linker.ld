MEMORY
{
	ITCM (rwx):  ORIGIN = 0x00000000, LENGTH = 512K
	DTCM (rwx):  ORIGIN = 0x20000000, LENGTH = 512K
	RAM (rwx):   ORIGIN = 0x20200000, LENGTH = 512K
	FLASH (rwx): ORIGIN = 0x60000000, LENGTH = 7936K
	ERAM (rwx):  ORIGIN = 0x70000000, LENGTH = 16384K
}

ENTRY(ImageVectorTable)

SECTIONS
{
	.text.headers : ALIGN(4) {
		FILL(0xFF)
		KEEP(*(.flashconfig))
		. = 0x1000;
		KEEP(*(.ivt))
		. = 0x1020;
		KEEP(*(.bootdata))
		. = ALIGN(1024);
	} > FLASH

	.text.code : ALIGN(4) {
        *(.flashmem*)
        *(.text*)       /* <== Now goes to flash */
        KEEP(*(.startup))
        . = ALIGN(4);
        KEEP(*(.init))
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;
        __init_array_start = .;
        KEEP (*(.init_array))
        __init_array_end = .;
        *(.text.__libc_init_array)
        . = ALIGN(4);
    } > FLASH

	.text.progmem : ALIGN(4) {
		*(.progmem*)
		. = ALIGN(4);
	} > FLASH

	.data.func : ALIGN(4) {
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.*__func__*)))
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.*__PRETTY_FUNCTION__*)))
		. = ALIGN(4);
	} > FLASH

	.text.fastrun : ALIGN(32) {
        . = . + 32; /* MPU to trap NULL pointer deref */
        *(.fastrun*)
        . = ALIGN(4);
    } > ITCM AT > FLASH

	.data : ALIGN(4) {
		*(.endpoint_queue)
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.rodata*)))
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.data*)))
		KEEP(*(.vectorsram))
		. = ALIGN(4);
	} > DTCM  AT> FLASH

	.bss : ALIGN(4) {
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.bss*)))
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(COMMON)))
		. = ALIGN(32);
		. = . + 32; /* MPU to trap stack overflow */
	} > DTCM

	.bss.dma (NOLOAD) : ALIGN(32) {
		*(.hab_log)
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.dmabuffers)))
		. = ALIGN(32);
	} > RAM

	.bss.padding (NOLOAD) : ALIGN(4) {
		_heap_start = . ;
		. = ABSOLUTE(ORIGIN(RAM) + LENGTH(RAM) - 128 - SIZEOF(.ARM.exidx));
		. = ALIGN(4);
		_heap_end = . ;
	} > RAM

	.ARM.exidx : ALIGN(4) {
		__exidx_start = .;
		*(.gnu.linkonce.armexidx.*)
		*(.ARM.exidx.fastrun*)
		*(.ARM.exidx.text*)
		*(.ARM.exidx)
		*(.ARM.exidx.flashmem*)
		*(.ARM.exidx.startup*)
		. = ALIGN(4);
		__exidx_end = .;
		*(.ARM.extab* .gnu.linkonce.armextab.*)
		. = ALIGN(4);
	} > FLASH

	.bss.extram (NOLOAD) : ALIGN(32) {
		*(SORT_BY_ALIGNMENT(SORT_BY_NAME(.externalram)))
		. = ALIGN(32);
	} > ERAM

	.text.csf : ALIGN(4) {
		FILL(0xFF)
		KEEP(*(.csf))
		__text_csf_end = .;
	} > FLASH

	_stext = ADDR(.text.fastrun);
    _etext = ADDR(.text.fastrun) + SIZEOF(.text.fastrun);
    _stextload = LOADADDR(.text.fastrun);

	_sdata = ADDR(.data);
	_edata = ADDR(.data) + SIZEOF(.data);
	_sdataload = LOADADDR(.data);

	_sexidx = ADDR(.ARM.exidx);
	_eexidx = ADDR(.ARM.exidx) + SIZEOF(.ARM.exidx);
	_sexidxload = LOADADDR(.ARM.exidx);

	_sbss = ADDR(.bss);
	_ebss = ADDR(.bss) + SIZEOF(.bss);

	_extram_start = ADDR(.bss.extram);
	_extram_end = ADDR(.bss.extram) + SIZEOF(.bss.extram);

	_itcm_block_count = (SIZEOF(.text.fastrun) + 0x7FFF) >> 15;
	_flexram_bank_config = 0xAAAAAAAA | ((1 << (_itcm_block_count * 2)) - 1);
	_estack = ORIGIN(DTCM) + ((16 - _itcm_block_count) << 15);

	_flashimagelen = __text_csf_end - ORIGIN(FLASH);
	_teensy_model_identifier = 0x25;

	.debug_info     0 : { *(.debug_info) }
	.debug_abbrev   0 : { *(.debug_abbrev) }
	.debug_line     0 : { *(.debug_line) }
	.debug_frame    0 : { *(.debug_frame) }
	.debug_str      0 : { *(.debug_str) }
	.debug_loc      0 : { *(.debug_loc) }

}
