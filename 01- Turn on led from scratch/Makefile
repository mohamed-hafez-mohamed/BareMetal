CC=arm-none-eabi-gcc
MACH=cortex-m3
CFLAGS= -c -mcpu=$(MACH) -mthumb -std=gnu99 -Wall -O0
LDFLAGS= -nostdlib -T stm32f10x_ls.ld -Wl,-Map=final.map

all:RCC_program.o GPIO_program.o lib.o main.o stm32f10x_startup.o final.elf final.hex final.bin

RCC_program.o:RCC_program.c
	$(CC) $(CFLAGS) RCC_program.c -o RCC_program.o
	
GPIO_program.o:GPIO_program.c
	$(CC) $(CFLAGS) GPIO_program.c -o GPIO_program.o
	
lib.o:lib.c
	$(CC) $(CFLAGS) lib.c -o lib.o
	
main.o:main.c
	$(CC) $(CFLAGS) main.c -o main.o
	
stm32f10x_startup.o:stm32f10x_startup.c
	$(CC) $(CFLAGS) stm32f10x_startup.c -o stm32f10x_startup.o
	
final.elf:RCC_program.o GPIO_program.o main.o stm32f10x_startup.o
	$(CC) $(LDFLAGS) RCC_program.o GPIO_program.o lib.o main.o stm32f10x_startup.o -o final.elf
	
final.hex:final.elf
	arm-none-eabi-objcopy -O ihex final.elf final.hex
	
final.bin:final.elf
	arm-none-eabi-objcopy -O binary final.elf final.bin

clean:
	rm -rf *.o *.elf *.hex *.bin *.map