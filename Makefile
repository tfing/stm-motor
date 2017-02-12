LIBROOT=../stm-lib
CORE=$(LIBROOT)/CMSIS
DEVICE=$(LIBROOT)/CMSIS/device
DRIVER=$(LIBROOT)/StdPeriph_Driver

# search path for library 
vpath %.c $(LIBROOT) 
vpath %.c $(DEVICE)/src 
vpath %.c $(DRIVER)/src 


CROSS_COMPILE ?= arm-none-eabi-
CC=$(CROSS_COMPILE)

CFLAGS = -Wall -Werror -O0\
		 -mcpu=cortex-m4 -mthumb -nostartfiles \
		 -I$(PWD) -I$(CORE)/inc -I$(DEVICE)/inc -I$(DRIVER)/inc \

ASFLAGS = --fatal-warnings \
		  -mcpu=cortex-m4 -mthumb \


.PHONY: all, flash, upload

all: motor.bin

system_stm32f4xx.o: $(DEVICE)/src/system_stm32f4xx.c
	$(CC)gcc $(CFLAGS) -c $(DEVICE)/src/system_stm32f4xx.c -o system_stm32f4xx.o 

stm32f4xx_rcc.o: $(DRIVER)/src/stm32f4xx_rcc.c
	$(CC)gcc $(CFLAGS) -c $^ -o $@ 

stm32f4xx_gpio.o: $(DRIVER)/src/stm32f4xx_gpio.c
	$(CC)gcc $(CFLAGS) -c $^ -o $@ 

stm32f4xx_exti.o: $(DRIVER)/src/stm32f4xx_exti.c
	$(CC)gcc $(CFLAGS) -c $^ -o $@ 

misc.o: $(DRIVER)/src/misc.c
	$(CC)gcc $(CFLAGS) -c $^ -o $@ 

stm32f4xx_syscfg.o: $(DRIVER)/src/stm32f4xx_syscfg.c
	$(CC)gcc $(CFLAGS) -c $^ -o $@ 

motor.bin : motor.c startup_stm32f427_437xx.s system_stm32f4xx.o stm32f4xx_rcc.o stm32f4xx_gpio.o stm32f4xx_exti.o misc.o stm32f4xx_syscfg.o
	$(CC)gcc $(CFLAGS) -c motor.c -o motor.o 
	$(CC)as  $(ASFLAGS) startup_stm32f427_437xx.s -o startup.o
	$(CC)ld -Map motor.map -T motor.ld -o motor.out motor.o startup.o system_stm32f4xx.o stm32f4xx_rcc.o stm32f4xx_gpio.o stm32f4xx_exti.o misc.o stm32f4xx_syscfg.o
	$(CC)objcopy -O binary motor.out motor.bin
	$(CC)objdump -h -D motor.out > motor.list

clean:
	rm -f *.o *.out *.bin *.list

flash: upload
up: upload
upload: motor.bin
	openocd -f board/stm32f446discovery.cfg -c "init" -c "reset halt" -c "flash write_image erase motor.bin 0x08000000" -c "verify_image motor.bin" -c "reset run" -c "shutdown" 

