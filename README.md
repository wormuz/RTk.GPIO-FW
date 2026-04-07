# RTk.GPIO Firmware & C++ Library

Firmware and C++ interface for the RTk.GPIO board — a Raspberry Pi-like GPIO interface for PC/Mac/Linux via USB serial.

Based on original work by [bit-hack](https://github.com/bit-hack/RTk.GPIO) (C++ library, v2.0 firmware) and [Ryanteck](https://bitbucket.org/ryanteckltd/rtk.gpio-mcu-firmware) (v1 firmware + mbed HAL).

**Current firmware: v2.2** — GPIO + PWM + SPI + Pull + Reset.

## Hardware

- **MCU:** STM32F030C8T6 (Cortex-M0, 48MHz, 64KB flash, 48-pin)
- **USB:** CH340G serial adapter, 230400 baud
- **GPIO:** 28 pins (GP0–GP27)
- **Crystal:** 12 MHz HSE (firmware uses HSI fallback)

---

## Firmware Versions

| Version | File | Size | Features |
|---------|------|------|----------|
| **v2.2** | `firmware/rtk_gpio_v2.2.bin` | 28.5 KB | GPIO, PWM (TIM3), SPI, Pull, Reset, Version |
| v2.1 | `firmware/rtk_gpio_v2.1.bin` | 27.5 KB | GPIO, SPI, Pull, Reset, Version (no PWM) |
| v2.0 | `firmware/firmware_10_04_22.bin` | 30.5 KB | Original bit-hack. **Broken on 12MHz boards** (assumes 8MHz HSE) |

### v2.2 Changes (2026-04-07)
- **PWM** via raw TIM3 registers — no mbed PwmOut (avoids TIM1/us_ticker conflict)
- 6 PWM-capable pins: GP8, GP9, GP10, GP18, GP25, GP26
- Protocol: `W` + 4 hex (freq), `P` + 2 hex (duty), `X` (stop)
- Response `K` (ok) or `E` (error/unsupported pin)

### v2.1 Changes
- Merged v1 mbed-dev base (HSI clock) + v2 features (SPI, Reset, auto-increment)
- `\r\n` on read response (v1 compatibility)
- `V` command resets state machine (prevents stale SPI state)
- Compiled locally with GCC ARM (no Keil Studio dependency)

### Why v2.0 doesn't work on RTk.GPIO boards
v2.0 was compiled in Keil Studio for NUCLEO-F030R8 (8MHz HSE crystal). RTk.GPIO board has **12MHz HSE** → PLL produces 72MHz (exceeds 48MHz max) → UART baud rate wrong → no serial communication. v2.1+ uses v1 mbed-dev which falls back to HSI (internal oscillator).

---

## Protocol (v2.2)

Pin char = `chr(pin + ord('a'))`: GP0=`a`, GP27=`|`.

| Command | Format | Response | Description |
|---------|--------|----------|-------------|
| Input | `eI` | — | GP4 input mode |
| Output | `eO` | — | GP4 output mode |
| Write | `e1` / `e0` | — | HIGH / LOW |
| Read | `e?` | `e1\r\n` | read pin state |
| Pull | `eU`/`eD`/`eN` | — | pull up/down/none |
| **PWM start** | `jW01F4` | `K` | GP9 PWM at 0x01F4=500Hz |
| **PWM duty** | `jP80` | — | GP9 duty 50% (0x80/0xFF) |
| **PWM stop** | `jX` | — | stop PWM |
| SPI | `~A5` | `XX` | transfer byte |
| Version | `V` | `RTk.GPIO v2.2...\r\n` | firmware version |
| Reset | `R` | `OK` | soft reset all |

Auto-increment: after each pin command, latched pin increments. Send pin char to re-latch.

---

## Pin Map

```
GP0 =PA1   GP7 =PF1   GP14=PA2   GP21=PB10
GP1 =PB12  GP8 =PB5*  GP15=PA3   GP22=PA11
GP2 =PB7   GP9 =PA6*  GP16=PB8   GP23=PB2
GP3 =PB6   GP10=PA7*  GP17=PB15  GP24=PB3
GP4 =PA8   GP11=PA5   GP18=PB1*  GP25=PB4*
GP5 =PA12  GP12=PF0   GP19=PA15  GP26=PB0*
GP6 =PA13  GP13=PA14  GP20=PB9   GP27=PB14

* = PWM capable (TIM3)
```

**40-pin header:**
```
pin1               3v3  ||  5v
         W8    ft  GP2  ||  5v
         W9    ft  GP3  ||  GND
         W7    ft  GP4  ||  GP14  --   W15
                   GND  ||  GP15  --   W16
         W0    ft  GP17 ||  GP18* --   W1
         W2    ft  GP27 ||  GND
         W3    ft  GP22 ||  GP23  ft   W4
                   3v3  ||  GP24  ft   W5
  mosi   W12   --  GP10*||  GND
  miso   W13   --  GP9* ||  GP25* ft   W6
  sck    W14   --  GP11 ||  GP8*  ft   W10
                   GND  ||  GP7   ft   W11
         W30   --  GP0  ||  GP1   ft   W31
         W21   ft  GP5  ||  GND
  swdio  W22   ft  GP6  ||  GP12  ft  W26
  swclk  W23   ft  GP13 ||  GND
         W24   ft  GP19 ||  GP16  ft  W27
         W25   --  GP26*||  GP20  ft  W28
                   GND  ||  GP21  ft  W29       pin40
```

---

## Flashing

**Requirements:** ST-LINK V2 (with NRST line), `openocd`

```bash
openocd -f interface/stlink.cfg -f target/stm32f0x.cfg \
  -c "reset_config srst_only connect_assert_srst" \
  -c "init" -c "reset halt" \
  -c "flash write_image erase firmware/rtk_gpio_v2.2.bin 0x08000000" \
  -c "verify_image firmware/rtk_gpio_v2.2.bin 0x08000000" \
  -c "reset run" -c "shutdown"
```

**SWD header:** 1=SWDIO (GP6), 2=SWCLK (GP13), 3=RST, 4=GND

Note: original ST-LINK V2 dongle may not work (no NRST control). ST-LINK V2 clone with NRST confirmed working.

## Building from Source

**Requirements:** `arm-none-eabi-gcc`, v1 mbed-dev repo

```bash
git clone https://bitbucket.org/ryanteckltd/rtk.gpio-mcu-firmware.git
cp firmware/firmware_v2.2.cpp rtk.gpio-mcu-firmware/main.cpp
cp firmware/Makefile rtk.gpio-mcu-firmware/
cd rtk.gpio-mcu-firmware
make
# Output: rtk_gpio_v2.1.bin
```

---

## C++ Library

Two APIs for host-side control (original bit-hack library):
- **gpio.h** — direct GPIO control
- **WiringPi.h** — WiringPi compatibility layer

See [examples/](examples/README.md) for usage.

## Python Library

Full-featured Python driver with dual v1/v2 auto-detect:
**https://github.com/wormuz/RTk.GPIO**

## References

- [Original v1 firmware source](https://bitbucket.org/ryanteckltd/rtk.gpio-mcu-firmware)
- [bit-hack C++ library](https://github.com/bit-hack/RTk.GPIO)
- [STM32F030C8T6 Datasheet](https://www.mouser.co.uk/datasheet/2/389/dm00088500-1797910.pdf)
- [Python library](https://github.com/wormuz/RTk.GPIO)
