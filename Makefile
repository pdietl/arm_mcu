TARGET       := prog1
PYOCD_TARGET := --target STM32G474RETx
CROSS        := arm-none-eabi-
CC           := $(CROSS)gcc
LD           := $(CROSS)ld
LDFLAGS      := -g -Ttext=0x8000000
ASFLAGS      := -g

$(TARGET).elf: $(TARGET).o
	$(LD) $(LDFLAGS) $< -o $@

.PHONY: gdbserver
gdbserver:
	pyocd gdbserver $(PYOCD_TARGET)

.PHONY: flash
flash: $(TARGET).elf
	pyocd flash $(PYOCD_TARGET) $<

.PHONY: erase
erase:
	pyocd erase --chip $(PYOCD_TARGET)

.PHONY: clean
clean:
	$(RM) $(TARGET).{o,elf}

.PHONY: dump
dump: $(TARGET).elf
	$(CROSS)objdump -D $<
