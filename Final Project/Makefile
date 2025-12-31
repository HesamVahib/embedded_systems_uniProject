######################################
# Project
######################################
TARGET = firmware
BUILD  = build

######################################
# MCU
######################################
MCU        = cortex-m4
FPU        = -mfpu=fpv4-sp-d16
FLOAT_ABI  = -mfloat-abi=hard
MCU_DEF    = STM32F411xE

######################################
# Toolchain
######################################
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc
CP = arm-none-eabi-objcopy
SZ = arm-none-eabi-size

######################################
# Paths
######################################
CORE     = Core
DRIVERS  = Drivers
STARTUP  = $(CORE)/Startup

######################################
# Source files
######################################
C_SOURCES = \
$(CORE)/Src/main.c \
$(CORE)/Src/app.c \
$(CORE)/Src/stm32f4xx_it.c \
$(CORE)/Src/stm32f4xx_hal_msp.c \
$(CORE)/Src/system_stm32f4xx.c \
$(CORE)/Src/syscalls.c \
$(CORE)/Src/sysmem.c \
$(wildcard $(DRIVERS)/STM32F4xx_HAL_Driver/Src/*.c)

ASM_SOURCES = \
$(STARTUP)/startup_stm32f411retx.s

######################################
# Includes
######################################
C_INCLUDES = \
-I$(CORE)/Inc \
-I$(DRIVERS)/STM32F4xx_HAL_Driver/Inc \
-I$(DRIVERS)/STM32F4xx_HAL_Driver/Inc/Legacy \
-I$(DRIVERS)/CMSIS/Device/ST/STM32F4xx/Include \
-I$(DRIVERS)/CMSIS/Include

######################################
# Flags
######################################
CFLAGS  = -mcpu=$(MCU) -mthumb $(FPU) $(FLOAT_ABI)
CFLAGS += -O2 -Wall -ffunction-sections -fdata-sections
CFLAGS += -D$(MCU_DEF)
CFLAGS += $(C_INCLUDES)

ASFLAGS = $(CFLAGS)

LDFLAGS  = -TSTM32F411RETX_FLASH.ld
LDFLAGS += -mcpu=$(MCU) -mthumb $(FPU) $(FLOAT_ABI)
LDFLAGS += -Wl,--gc-sections

######################################
# Windows/Linux helpers
######################################
ifeq ($(OS),Windows_NT)
RM_BUILD    = powershell -NoProfile -Command "if (Test-Path '$(BUILD)') { Remove-Item -Recurse -Force '$(BUILD)' }"
MKDIR_BUILD = powershell -NoProfile -Command "New-Item -ItemType Directory -Force '$(BUILD)' | Out-Null"
else
RM_BUILD    = rm -rf $(BUILD)
MKDIR_BUILD = mkdir -p $(BUILD)
endif

######################################
# Build rules
######################################
OBJECTS = \
$(addprefix $(BUILD)/, $(notdir $(C_SOURCES:.c=.o))) \
$(addprefix $(BUILD)/, $(notdir $(ASM_SOURCES:.s=.o)))

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD)/$(TARGET).elf hex bin

$(BUILD):
	$(MKDIR_BUILD)

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s | $(BUILD)
	$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS)
	$(CC) $^ $(LDFLAGS) -o $@
	$(SZ) $@

hex: $(BUILD)/$(TARGET).elf
	$(CP) -O ihex $< $(BUILD)/$(TARGET).hex

bin: $(BUILD)/$(TARGET).elf
	$(CP) -O binary $< $(BUILD)/$(TARGET).bin

clean:
	$(RM_BUILD)

######################################
# END
######################################
