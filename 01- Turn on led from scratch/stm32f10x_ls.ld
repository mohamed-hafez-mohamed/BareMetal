ENTRY(Reset_Handler)

MEMORY
{
   FLASH(rx) : ORIGIN = 0x08002000 , LENGTH = 56k
   SRAM(rwx) : ORIGIN = 0x20000000 , LENGTH = 20k
}

SECTIONS
{
   .sharedlib :
   {
      *(.isr_vector)
      lib.o (.text)
      . = ALIGN(4);
      _etext = .;
   }> FLASH
   
   .text :
   {
      *(.isr_vector)
      RCC_program.o (.text)
      GPIO_program.o (.text)
      stm32f10x_startup.o (.text)
      main.o (.text)
      *(.rodata)
      . = ALIGN(4);
      _etext = .;
   }> FLASH
   
   .data :
   {
      _sdata = .;
      *(.data)
      . = ALIGN(4);
      _edata = .;
   }> SRAM AT> FLASH

   .bss :
   {
      _sbss = .;
      *(.bss)
      . = ALIGN(4);
      _ebss = .;
   }> SRAM

}