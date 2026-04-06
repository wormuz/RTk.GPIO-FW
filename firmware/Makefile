# RTk.GPIO firmware v2.1 — Makefile for GCC ARM
# Target: STM32F030C8T6 (48-pin, 64KB flash, 8KB SRAM)

CC      = arm-none-eabi-gcc
CXX     = arm-none-eabi-g++
AS      = arm-none-eabi-gcc
LD      = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

TARGET  = rtk_gpio_v2.1

# Directories
MBED    = mbed-dev
HAL_STM = $(MBED)/targets/hal/TARGET_STM/TARGET_STM32F0
CMSIS_STM = $(MBED)/targets/cmsis/TARGET_STM/TARGET_STM32F0
NUCLEO  = $(CMSIS_STM)/TARGET_NUCLEO_F030R8
TOOLCHAIN = $(NUCLEO)/TOOLCHAIN_GCC_ARM

# Linker script
LDSCRIPT = $(TOOLCHAIN)/STM32F030X8.ld

# CPU flags
CPU_FLAGS = -mcpu=cortex-m0 -mthumb -mfloat-abi=soft

# Defines
DEFINES = \
	-DTARGET_NUCLEO_F030R8 \
	-DTARGET_STM32F030R8 \
	-DTARGET_STM32F0 \
	-DTOOLCHAIN_GCC_ARM \
	-DTOOLCHAIN_GCC \
	-D__CORTEX_M0 \
	-DARM_MATH_CM0 \
	-D__MBED__=1 \
	-DSTM32F030x8

# Include paths
INCLUDES = \
	-I$(MBED)/api \
	-I$(MBED)/hal \
	-I$(MBED)/targets/cmsis \
	-I$(CMSIS_STM) \
	-I$(NUCLEO) \
	-I$(HAL_STM) \
	-I$(HAL_STM)/TARGET_NUCLEO_F030R8

# Compiler flags
COMMON_FLAGS = $(CPU_FLAGS) $(DEFINES) $(INCLUDES) -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -Wall
CFLAGS   = $(COMMON_FLAGS) -std=gnu11
CXXFLAGS = $(COMMON_FLAGS) -std=gnu++11 -fno-threadsafe-statics
ASFLAGS  = $(CPU_FLAGS) -x assembler-with-cpp $(DEFINES) $(INCLUDES)
LDFLAGS  = $(CPU_FLAGS) -T$(LDSCRIPT) -Wl,--gc-sections -Wl,-Map=$(TARGET).map --specs=nano.specs --specs=nosys.specs

# Source files
C_SRCS = \
	$(CMSIS_STM)/stm32f0xx_hal.c \
	$(CMSIS_STM)/stm32f0xx_hal_cortex.c \
	$(CMSIS_STM)/stm32f0xx_hal_dma.c \
	$(CMSIS_STM)/stm32f0xx_hal_flash.c \
	$(CMSIS_STM)/stm32f0xx_hal_flash_ex.c \
	$(CMSIS_STM)/stm32f0xx_hal_gpio.c \
	$(CMSIS_STM)/stm32f0xx_hal_i2c.c \
	$(CMSIS_STM)/stm32f0xx_hal_pwr.c \
	$(CMSIS_STM)/stm32f0xx_hal_pwr_ex.c \
	$(CMSIS_STM)/stm32f0xx_hal_rcc.c \
	$(CMSIS_STM)/stm32f0xx_hal_rcc_ex.c \
	$(CMSIS_STM)/stm32f0xx_hal_spi.c \
	$(CMSIS_STM)/stm32f0xx_hal_spi_ex.c \
	$(CMSIS_STM)/stm32f0xx_hal_tim.c \
	$(CMSIS_STM)/stm32f0xx_hal_tim_ex.c \
	$(CMSIS_STM)/stm32f0xx_hal_uart.c \
	$(CMSIS_STM)/stm32f0xx_hal_uart_ex.c \
	$(NUCLEO)/system_stm32f0xx.c \
	$(NUCLEO)/cmsis_nvic.c \
	$(NUCLEO)/hal_tick.c \
	$(HAL_STM)/TARGET_NUCLEO_F030R8/PeripheralPins.c \
	$(HAL_STM)/analogin_api.c \
	$(HAL_STM)/gpio_api.c \
	$(HAL_STM)/gpio_irq_api.c \
	$(HAL_STM)/i2c_api.c \
	$(HAL_STM)/mbed_overrides.c \
	$(HAL_STM)/pinmap.c \
	$(HAL_STM)/port_api.c \
	$(HAL_STM)/pwmout_api.c \
	$(HAL_STM)/rtc_api.c \
	$(HAL_STM)/serial_api.c \
	$(HAL_STM)/sleep.c \
	$(HAL_STM)/spi_api.c \
	$(HAL_STM)/us_ticker.c \
	$(MBED)/common/board.c \
	$(MBED)/common/gpio.c \
	$(MBED)/common/pinmap_common.c \
	$(MBED)/common/us_ticker_api.c \
	$(MBED)/common/ticker_api.c \
	$(MBED)/common/lp_ticker_api.c \
	$(MBED)/common/wait_api.c \
	$(MBED)/common/assert.c \
	$(MBED)/common/error.c \
	$(MBED)/common/mbed_interface.c \
	$(MBED)/common/rtc_time.c

CXX_SRCS = \
	main.cpp \
	$(MBED)/common/retarget.cpp \
	$(MBED)/common/Serial.cpp \
	$(MBED)/common/SerialBase.cpp \
	$(MBED)/common/RawSerial.cpp \
	$(MBED)/common/Stream.cpp \
	$(MBED)/common/FileLike.cpp \
	$(MBED)/common/FileBase.cpp \
	$(MBED)/common/FileSystemLike.cpp \
	$(MBED)/common/FilePath.cpp \
	$(MBED)/common/BusIn.cpp \
	$(MBED)/common/BusOut.cpp \
	$(MBED)/common/BusInOut.cpp \
	$(MBED)/common/InterruptIn.cpp \
	$(MBED)/common/InterruptManager.cpp \
	$(MBED)/common/CallChain.cpp \
	$(MBED)/common/Ticker.cpp \
	$(MBED)/common/Timeout.cpp \
	$(MBED)/common/Timer.cpp \
	$(MBED)/common/TimerEvent.cpp \
	$(MBED)/common/SPI.cpp \
	$(MBED)/common/SPISlave.cpp \
	$(MBED)/common/I2C.cpp \
	$(MBED)/common/I2CSlave.cpp

ASM_SRCS = $(TOOLCHAIN)/startup_stm32f030x8.S

# Object files
BUILD_DIR = build
C_OBJS   = $(addprefix $(BUILD_DIR)/, $(C_SRCS:.c=.o))
CXX_OBJS = $(addprefix $(BUILD_DIR)/, $(CXX_SRCS:.cpp=.o))
ASM_OBJS = $(addprefix $(BUILD_DIR)/, $(ASM_SRCS:.S=.o))
ALL_OBJS = $(C_OBJS) $(CXX_OBJS) $(ASM_OBJS)

.PHONY: all clean flash

all: $(TARGET).bin
	$(SIZE) $(TARGET).elf

$(TARGET).elf: $(ALL_OBJS)
	$(LD) $(LDFLAGS) -o $@ $^

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: %.S
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c -o $@ $<

clean:
	rm -rf $(BUILD_DIR) $(TARGET).elf $(TARGET).bin $(TARGET).map

flash: $(TARGET).bin
	openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
		-c "reset_config srst_only connect_assert_srst" \
		-c "init" -c "reset halt" \
		-c "flash write_image erase $(TARGET).bin 0x08000000" \
		-c "verify_image $(TARGET).bin 0x08000000" \
		-c "reset run" -c "shutdown"
