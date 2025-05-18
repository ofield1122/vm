# File: Makefile

# Toolchain
TOOLCHAIN_PATH := $(HOME)/Projects/toolchains/gcc-arm-none-eabi-10-2020-q4-major/bin
#CC = arm-none-eabi-gcc
CC := $(TOOLCHAIN_PATH)/arm-none-eabi-gcc
AS := $(TOOLCHAIN_PATH)/arm-none-eabi-as
LD := $(TOOLCHAIN_PATH)/arm-none-eabi-ld
OBJCOPY := $(TOOLCHAIN_PATH)/arm-none-eabi-objcopy
OBJDUMP := $(TOOLCHAIN_PATH)/arm-none-eabi-objdump

SD_VERSION := $(shell grep '#define SD_VERSION' main.h | sed 's/.*"\(.*\)"/\1/')
OUTPUT_BIN = NUC131-$(SD_VERSION).bin
# MCU-specific flags
CFLAGS = -mcpu=cortex-m0 -mthumb -O2 -Wall -ffreestanding -nostdlib -MMD -MP
CFLAGS += -ffunction-sections -fdata-sections
#LDFLAGS += -T ../Library/Device/Nuvoton/NUC131/Source/GCC/gcc_arm.ld -lgcc
LDFLAGS += -T ../Library/Device/Nuvoton/NUC131/Source/GCC/gcc_arm.ld -lgcc -Wl,-Map=output.map
LDFLAGS += -Wl,--gc-sections

# Files
SRCS = main.c ../Library/Device/Nuvoton/NUC131/Source/system_NUC131.c ../Library/Device/Nuvoton/NUC131/Source/GCC/startup_NUC131.S
SRCS += ../Library/StdDriver/src/clk.c
SRCS += ../Library/StdDriver/src/uart.c
SRCS += ../Library/StdDriver/src/sys.c
SRCS += ../Library/StdDriver/src/gpio.c
SRCS += ../Library/StdDriver/src/retarget.c

OBJS = $(SRCS:.c=.o)
TARGET = output.elf

# Include paths
INCLUDES = -I../Library/CMSIS/Include \
           -I../Library/StdDriver/inc \
           -I../Library/Device/Nuvoton/NUC131/Include

all: $(TARGET)

%.o: %.c
	@mkdir -p build
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
	

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@
	$(OBJCOPY) -O binary $@ $(OUTPUT_BIN)
	$(OBJDUMP) -d $@ > output.lst

clean:
	rm -f *.o *.elf *.bin *.lst *.d
	
# Automatically include dependency files
#-include $(OBJS:.o=.d)
	
