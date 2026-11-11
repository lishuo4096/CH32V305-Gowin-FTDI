# ---------------------------------------------------------------------------
# Command line build for the CH32V305 "Alternative_FT232H" firmware.
#
#   make            -> build/Alternative_FT232H.hex (+ .elf / .map / .lst)
#   make clean      -> remove the build directory
#
# Toolchain: MounRiver Studio 2 bundled RISC-V GCC (riscv-none-embed-).
# The MounRiver IDE itself also builds this project; this file only exists so
# the firmware can be compiled from a plain command line.
# ---------------------------------------------------------------------------

TC        = C:/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC/bin
CC        = "$(TC)/riscv-none-embed-gcc.exe"
OBJCOPY   = "$(TC)/riscv-none-embed-objcopy.exe"
OBJDUMP   = "$(TC)/riscv-none-embed-objdump.exe"
SIZE      = "$(TC)/riscv-none-embed-size.exe"

TARGET    = Alternative_FT232H
BUILD     = build

CFLAGS    = -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore \
            -fmax-errors=20 -O2 -fmessage-length=0 -fsigned-char \
            -ffunction-sections -fdata-sections -fno-common \
            -Wunused -Wuninitialized -g -std=gnu99 \
            -IUser -IPeripheral/inc -ICore -IDebug

LDFLAGS   = -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore \
            -T Ld/Link.ld -nostartfiles -Xlinker --gc-sections \
            -Wl,-Map,"$(BUILD)/$(TARGET).map" --specs=nano.specs --specs=nosys.specs

C_SRCS    = $(wildcard Peripheral/src/*.c) $(wildcard Core/*.c) $(wildcard Debug/*.c) $(wildcard User/*.c)
S_SRCS    = Startup/startup_ch32v30x_D8C.S

C_OBJS    = $(patsubst %.c,$(BUILD)/%.o,$(C_SRCS))
S_OBJS    = $(patsubst %.S,$(BUILD)/%.o,$(S_SRCS))
OBJS      = $(C_OBJS) $(S_OBJS)

MKDIR     = if not exist "$(subst /,\,$(dir $@))" mkdir "$(subst /,\,$(dir $@))"

.PHONY: all clean size

all: $(BUILD)/$(TARGET).hex

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@
	$(OBJCOPY) -O binary $< $(BUILD)/$(TARGET).bin
	$(OBJDUMP) -M xw -S $< > $(BUILD)/$(TARGET).lst
	@echo "=== build ok: $@ ==="

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) --format=berkeley $@

$(BUILD)/%.o: %.c
	@$(MKDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.S
	@$(MKDIR)
	$(CC) $(CFLAGS) -c $< -o $@

size: $(BUILD)/$(TARGET).elf
	$(SIZE) --format=berkeley $<

clean:
	-rmdir /s /q $(subst /,\,$(BUILD))
