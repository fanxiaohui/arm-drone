
STM32_ROOT=/usr/local/STM32Cube_FW_F1_V1.6.0
LL_INC=$(STM32_ROOT)/Drivers/STM32F1xx_HAL_Driver/Inc
LL_SRC=$(STM32_ROOT)/Drivers/STM32F1xx_HAL_Driver/Src

AS=arm-as
CC=arm-gcc
CFLAGS=-MMD -Wall -specs=nano.specs -specs=nosys.specs -DSTM32F103xB -DHAVE_ASSERT_FUNC -mthumb -mcpu=cortex-m3 -ffunction-sections -fdata-sections -I$(STM32_ROOT)/Drivers/CMSIS/Include -I$(STM32_ROOT)/Drivers/CMSIS/Device/ST/STM32F1xx/Include -I$(LL_INC) -I.. -finline-functions -ggdb3 # -fverbose-asm -save-temps
LDFLAGS=-Wl,-Tstm32f103x8.ld -Wl,--gc-sections
VPATH=$(LL_SRC) ../os

OBJECTS = startup_stm32f103x8.o system_stm32f1xx.o \
	main.o scheduler.o assert_func.o console.o utils.o exti.o buttons.o spi.o \
	gpio_stm32f1xx.c nrf24l01p.c

controller.elf: $(OBJECTS)
	$(LINK.c) -o $@ $^ -Wl,-Map=$@.map $(LDLIBS)

controller.bin: controller.elf
	arm-objcopy -O binary $^ $@

upload: controller.elf
	openocd -f openocd.cfg -c "program controller.elf verify reset exit"

clean:
	-rm *.d *.o *.elf*

include $(wildcard $(OBJECTS:.o=.d))
