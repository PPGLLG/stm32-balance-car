# =============================================================================
# STM32F103C8T6 - CubeMX HAL Cross-platform Makefile
# Supports: macOS + VS Code + GCC / Windows + VS Code + GCC / Windows + Keil
#
# Designed to coexist with CubeMX: regenerating code in CubeMX will NOT
# touch this Makefile, the linker script, .vscode/, or any GCC build output.
#
# Works anywhere arm-none-eabi-gcc + make are available.
# =============================================================================

TOOLCHAIN := arm-none-eabi-
CC       := $(TOOLCHAIN)gcc
OBJCOPY  := $(TOOLCHAIN)objcopy
SIZE     := $(TOOLCHAIN)size

CPU      := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
CFLAGS   := $(CPU) -Wall -Wextra -O0 -g -ffreestanding -std=gnu99 \
            -ffunction-sections -fdata-sections -MD -MP
DEFINES  := -DSTM32F103xB -DUSE_HAL_DRIVER \
            -DCMSIS_device_header=\"stm32f1xx.h\" \
            -DUSE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION=1

# ---------------------------------------------------------------------------
# Include paths — mirrors CubeMX project structure
# ---------------------------------------------------------------------------
INCLUDES := \
            -ICore/Inc \
            -IDrivers/STM32F1xx_HAL_Driver/Inc \
            -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
            -IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
            -IDrivers/CMSIS/Core/Include \
            -IMiddlewares/Third_Party/FreeRTOS/Source/include \
            -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
            -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 \
            -ITools/gcc_support/FreeRTOS \
            -IApp \
            -ITask \
            -IBSP/ADC \
            -IBSP/ESP01 \
            -IBSP/IR \
            -IBSP/MOTOR \
            -IBSP/MPU6050 \
            -IBSP/OLED \
            -IBSP/SOFT_I2C

LINKER_SCRIPT := STM32F103C8T6.ld

# Bare-metal linking: Core/Src/syscalls.c provides syscall stubs.
# -nostdlib keeps libc out; -lgcc pulls in compiler helpers (__aeabi_* etc.).
LDFLAGS  := $(CPU) -T $(LINKER_SCRIPT) -nostdlib \
            -Wl,-Map=build/output.map,--gc-sections

ALL_CFLAGS = $(CFLAGS) $(DEFINES) $(INCLUDES)

# ---------------------------------------------------------------------------
# Auto-detect source files — no need to maintain lists manually
# ---------------------------------------------------------------------------
# Exclude *_template.c — those are reference templates, not meant to be compiled
HAL_SOURCES := $(filter-out %_template.c, $(wildcard Drivers/STM32F1xx_HAL_Driver/Src/*.c))

# FreeRTOS kernel sources
# - Core .c files in Source/ root (tasks, queue, list, timers, event_groups,
#   stream_buffer, croutine).  croutine.c is empty when configUSE_CO_ROUTINES=0,
#   so it compiles to nothing — linker will garbage-collect it.
# - CMSIS-RTOS V2 wrapper
# - heap_4.c (selected by USE_FreeRTOS_HEAP_4 in FreeRTOSConfig.h)
# - ARM_CM3 GCC port (NOT the RVDS port used by Keil)
FREERTOS_DIR := Middlewares/Third_Party/FreeRTOS/Source
FREERTOS_SOURCES := $(wildcard $(FREERTOS_DIR)/*.c) \
                    $(FREERTOS_DIR)/CMSIS_RTOS_V2/cmsis_os2.c \
                    $(FREERTOS_DIR)/portable/MemMang/heap_4.c \
                    Tools/gcc_support/FreeRTOS/port.c

C_SOURCES   := $(wildcard Core/Src/*.c Core/Src/**/*.c) \
               $(wildcard BSP/*/*.c) \
               $(wildcard App/*.c) \
               $(wildcard Task/*.c) \
               $(HAL_SOURCES) $(FREERTOS_SOURCES)
ASM_SOURCES := Tools/gcc_support/startup/startup_stm32f103xb.s

C_OBJECTS   := $(C_SOURCES:%=build/%.o)
ASM_OBJECTS := $(ASM_SOURCES:%=build/%.o)
ALL_OBJECTS := $(C_OBJECTS) $(ASM_OBJECTS)

# =============================================================================
# OS Detection
# =============================================================================
ifeq ($(OS),Windows_NT)
  DETECTED_OS := Windows
  RM   := rm -rf
  MKDIR := mkdir -p
else
  DETECTED_OS := $(shell uname -s)
  RM   := rm -rf
  MKDIR := mkdir -p
endif

# ---------------------------------------------------------------------------
# macOS: prefer official ARM GNU Toolchain (includes newlib headers).
# Homebrew's arm-none-eabi-gcc formula ships --without-headers, so it can't
# compile FreeRTOS which includes <stdlib.h>. The official build from
# developer.arm.com (or the gcc-arm-embedded cask) is the canonical choice.
# On Windows/Linux the standard arm-none-eabi- from PATH is used.
# ---------------------------------------------------------------------------
ifeq ($(DETECTED_OS),Darwin)
  ARM_OFFICIAL := /opt/homebrew/arm-gnu-toolchain/15.2.rel1/bin
  ifneq ($(wildcard $(ARM_OFFICIAL)/arm-none-eabi-gcc),)
    TOOLCHAIN := $(ARM_OFFICIAL)/arm-none-eabi-
    CC        := $(TOOLCHAIN)gcc
    OBJCOPY   := $(TOOLCHAIN)objcopy
    SIZE      := $(TOOLCHAIN)size
  endif
endif

# Flash tools (paths can be overridden via environment)
ST_FLASH    ?= st-flash
OPENOCD     ?= openocd
STLINK_CLI  ?= ST-LINK_CLI.exe

.PHONY: all clean flash flash-stlink flash-openocd help

# =============================================================================
# Default target  (must come before -include so it stays the default goal)
# =============================================================================
all: build/output.hex

# Auto-generated dependency files (recompile when headers change)
# Keep AFTER the "all:" target — otherwise the first target in the first
# included .d file hijacks the default goal.
-include $(ALL_OBJECTS:.o=.d)

# =============================================================================
# Compile .c files (preserving directory structure inside build/)
# =============================================================================
build/%.c.o: %.c
	@$(MKDIR) $(dir $@)
	$(CC) $(ALL_CFLAGS) -c $< -o $@

# =============================================================================
# Assemble .s files (GAS syntax — CubeMX already ships GCC startup files)
# =============================================================================
build/%.s.o: %.s
	@$(MKDIR) $(dir $@)
	$(CC) $(ALL_CFLAGS) -c $< -o $@

# =============================================================================
# Link
# =============================================================================
build/output.elf: $(ALL_OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@ -lgcc -lm
	$(SIZE) $@

build/output.hex: build/output.elf
	$(OBJCOPY) -O ihex $< $@

# =============================================================================
# Flash targets
# =============================================================================

# st-flash (macOS default, also works on Linux / Windows if installed)
flash: all
	$(ST_FLASH) --format ihex write build/output.hex

# OpenOCD via ST-Link (recommended for Windows GCC users)
flash-openocd: all
	$(OPENOCD) -f interface/stlink.cfg -f target/stm32f1x.cfg \
		-c "program build/output.hex verify reset exit"

# ST-LINK CLI (Windows, comes with Keil MDK or STM32 ST-LINK Utility)
# Usage: make flash-stlink STLINK_CLI="C:/Keil_v5/ARM/STLink/ST-LINK_CLI.exe"
flash-stlink: all
	$(STLINK_CLI) -c SWD -P build/output.hex -V -Rst

# =============================================================================
# Clean
# =============================================================================
clean:
	$(RM) build

# =============================================================================
# Help
# =============================================================================
help:
	@echo "============================================"
	@echo "STM32F103C8T6 CubeMX HAL Project - Build Targets"
	@echo "Detected OS: $(DETECTED_OS)"
	@echo "============================================"
	@echo ""
	@echo "Build:"
	@echo "  make              - Build firmware (output: build/output.hex)"
	@echo "  make clean        - Remove build artifacts"
	@echo ""
	@echo "Flash (choose based on your platform & tools):"
	@echo "  make flash            - Flash via st-flash (macOS/Linux default)"
	@echo "  make flash-openocd    - Flash via OpenOCD (recommended for Windows)"
	@echo "  make flash-stlink     - Flash via ST-LINK_CLI (Windows, from Keil/ST)"
	@echo ""
	@echo "Custom tool paths (if not in PATH):"
	@echo "  make flash ST_FLASH=/path/to/st-flash"
	@echo "  make flash-openocd OPENOCD=/path/to/openocd"
	@echo "  make flash-stlink STLINK_CLI=\"C:/Keil_v5/ARM/STLink/ST-LINK_CLI.exe\""
	@echo "============================================"
