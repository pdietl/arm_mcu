PYOCD_TARGET := --target STM32G474RETx
CROSS        := arm-none-eabi-
CC           := $(CROSS)gcc
LD           := $(CROSS)ld
LDFLAGS      := -g -T stm32g474re.ld
ASFLAGS      := -g
OBJS         := vector_table.o

blinky.elf:

%.elf: %.o $(OBJS)
	$(LD) $(LDFLAGS) $^ -o $*.elf

.PHONY: gdbserver
gdbserver:
	pyocd gdbserver $(PYOCD_TARGET)

flash-%: %.elf
	pyocd flash $(PYOCD_TARGET) $<

.PHONY: erase
erase:
	pyocd erase --chip $(PYOCD_TARGET)

.PHONY: clean
clean:
	$(RM) *.elf *.o

.PHONY: dump
dump-%: %.elf
	$(CROSS)objdump -D $<
