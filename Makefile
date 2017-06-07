
STM32_ROOT=/usr/local/STM32Cube_FW_F0_V1.7.0
LL_INC=$(STM32_ROOT)/Drivers/STM32F0xx_HAL_Driver/Inc
LL_SRC=$(STM32_ROOT)/Drivers/STM32F0xx_HAL_Driver/Src

AS=arm-as
CC=arm-gcc
CFLAGS=-specs=nano.specs -specs=nosys.specs -DSTM32F030x6 -DHAVE_ASSERT_FUNC -mthumb -mcpu=cortex-m0 -ffunction-sections -fdata-sections -I$(STM32_ROOT)/Drivers/CMSIS/Include -I$(STM32_ROOT)/Drivers/CMSIS/Device/ST/STM32F0xx/Include -I$(LL_INC) -fverbose-asm -save-temps -finline-functions -ggdb3
LDFLAGS=-Wl,-Tstm32f030f4.ld -Wl,--gc-sections
VPATH=$(LL_SRC)

OBJECTS = startup_stm32f030x6.o system_stm32f0xx.o stm32f0xx_ll_gpio.o \
	led.o scheduler.o assert_func.o

led.elf: $(OBJECTS)
	$(LINK.c) -o $@ $^ -Wl,-Map=$@.map $(LDLIBS)

led.bin: led.elf
	arm-objcopy -O binary $^ $@

upload: led.elf
	openocd -f openocd.cfg -c "program led.elf verify reset exit"

# dependencies
scheduler.o: scheduler.c scheduler.h
