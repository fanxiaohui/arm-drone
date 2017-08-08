
STM32_ROOT=/usr/local/STM32Cube_FW_F0_V1.7.0
LL_INC=$(STM32_ROOT)/Drivers/STM32F0xx_HAL_Driver/Inc
LL_SRC=$(STM32_ROOT)/Drivers/STM32F0xx_HAL_Driver/Src

AS=arm-as
CC=arm-gcc
CFLAGS=-MMD -Wall -specs=nano.specs -specs=nosys.specs -DSTM32F030x6 -DHAVE_ASSERT_FUNC -mthumb -mcpu=cortex-m0 -ffunction-sections -fdata-sections -I$(STM32_ROOT)/Drivers/CMSIS/Include -I$(STM32_ROOT)/Drivers/CMSIS/Device/ST/STM32F0xx/Include -I$(LL_INC) -I.. -finline-functions -ggdb3 # -fverbose-asm -save-temps
LDFLAGS=-Wl,-Tstm32f030f4.ld -Wl,--gc-sections
VPATH=$(LL_SRC) ../os

OBJECTS = startup_stm32f030x6.o system_stm32f0xx.o \
	main.o scheduler.o assert_func.o console.o utils.o exti.o buttons.o spi.o \
	nrf24l01p.c

remote.elf: $(OBJECTS)
	$(LINK.c) -o $@ $^ -Wl,-Map=$@.map $(LDLIBS)

remote.bin: remote.elf
	arm-objcopy -O binary $^ $@

upload: remote.elf
	openocd -f openocd.cfg -c "program remote.elf verify reset exit"

clean:
	-rm *.d *.o *.elf*

include $(wildcard $(OBJECTS:.o=.d))
