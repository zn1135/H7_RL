/* Linker layout for STM32H723VGTx. DMA-visible buffers use AXI SRAM. */
ENTRY(Reset_Handler)

_Min_Heap_Size = 0x800;
_Min_Stack_Size = 0x1000;
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);

MEMORY
{
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  AXIRAM  (xrw) : ORIGIN = 0x24000000, LENGTH = 304K
  USBRAM  (xrw) : ORIGIN = 0x2404C000, LENGTH = 16K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text .text.*)
    *(.glue_7 .glue_7t)
    *(.eh_frame*)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata .rodata.*)
    . = ALIGN(4);
  } > FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    __exidx_end = .;
  } > FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH
  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH

  .usb_data :
  {
    . = ALIGN(32);
    _susb_data = .;
    *usb_device.o(.data .data.*)
    *usbd_*.o(.data .data.*)
    . = ALIGN(32);
    _eusb_data = .;
  } > USBRAM AT > FLASH
  _siusb_data = LOADADDR(.usb_data);

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data .data.*)
    . = ALIGN(4);
    _edata = .;
  } > DTCMRAM AT > FLASH

  .usb_bss (NOLOAD) :
  {
    . = ALIGN(32);
    _susb_bss = .;
    *usb_device.o(.bss .bss.* COMMON)
    *usbd_*.o(.bss .bss.* COMMON)
    . = ALIGN(32);
    _eusb_bss = .;
  } > USBRAM

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss .bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > AXIRAM

  .bss_axi (NOLOAD) :
  {
    . = ALIGN(32);
    _sbss_axi = .;
    *(.s2r_dma)
    *Vofa_send.o(.bss .bss.* COMMON)
    *uart_idle.o(.bss .bss.* COMMON)
    . = ALIGN(32);
    _ebss_axi = .;
  } > AXIRAM

  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } > DTCMRAM

  /DISCARD/ : { *(.note.GNU-stack) }
}
