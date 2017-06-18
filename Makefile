
STM32_ROOT=/usr/local/STM32Cube_FW_F0_V1.7.0
LL_INC=$(STM32_ROOT)/Drivers/STM32F0xx_HAL_Driver/Inc
LL_SRC=$(STM32_ROOT)/Drivers/STM32F0xx_HAL_Driver/Src

AS=arm-as
CC=arm-gcc
CFLAGS=-Wall -specs=nano.specs -specs=nosys.specs -DSTM32F030x6 -DHAVE_ASSERT_FUNC -mthumb -mcpu=cortex-m0 -ffunction-sections -fdata-sections -I$(STM32_ROOT)/Drivers/CMSIS/Include -I$(STM32_ROOT)/Drivers/CMSIS/Device/ST/STM32F0xx/Include -I$(LL_INC) -fverbose-asm -save-temps -finline-functions -ggdb3
LDFLAGS=-Wl,-Tstm32f030f4.ld -Wl,--gc-sections
VPATH=$(LL_SRC)

OBJECTS = startup_stm32f030x6.o system_stm32f0xx.o stm32f0xx_ll_gpio.o \
	main.o scheduler.o assert_func.o console.o utils.o exti.o buttons.o

remote.elf: $(OBJECTS)
	$(LINK.c) -o $@ $^ -Wl,-Map=$@.map $(LDLIBS)

remote.bin: remote.elf
	arm-objcopy -O binary $^ $@

upload: remote.elf
	openocd -f openocd.cfg -c "program remote.elf verify reset exit"

# dependencies
main.o: main.c scheduler.h exti.h buttons.h
scheduler.o: scheduler.c scheduler.h utils.h
console.o: console.c console.h utils.h
utils.o: utils.c utils.h
exti.o: exti.c utils.h scheduler.h exti.h
buttons.o: buttons.c buttons.h scheduler.h exti.h utils.h
