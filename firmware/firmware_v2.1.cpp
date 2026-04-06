/*
 * RTk.GPIO Microcontroller Firmware v2.1
 * Based on original v1 by Ryanteck LTD (C) 2016, GNU GPL V3
 * v2 protocol features from bit-hack/RTk.GPIO (C) 2022
 * v2.1 merged + bug fixes 2026-04-07
 *
 * Changes from v1:
 *   - Reset command 'R' → "OK" (v2 enhanced mode detection)
 *   - SPI hardware transfer '~' + hex byte → hex response
 *   - Auto-increment latched pin after each command
 *   - Read response includes \r\n (v1 compat)
 *   - Pull up/down/none: U/D/N (already in v1)
 *   - V command fixed: only needs 1 byte (no blocking getc)
 *   - I2C removed (conflicts with GPIO, rarely used)
 *
 * Target: STM32F030C8T6 (48-pin, 12MHz HSE crystal)
 * Board: RTk.GPIO
 * mbed: v1 mbed-dev (HSI fallback, proven stable)
 */

#include "mbed.h"

#define BAUD_RATE   230400
#define PIN_COUNT   28
#define VERSION_STR "RTk.GPIO v2.1 2026-04-07"

/* UART on PA_9 (TX) / PA_10 (RX) — connected to CH340G */
Serial serialPort(SERIAL_TX, SERIAL_RX);

/* All 28 GPIO pins */
DigitalInOut IO0(GPIO0);
DigitalInOut IO1(GPIO1);
DigitalInOut IO2(GPIO2);
DigitalInOut IO3(GPIO3);
DigitalInOut IO4(GPIO4);
DigitalInOut IO5(GPIO5);
DigitalInOut IO6(GPIO6);
DigitalInOut IO7(GPIO7);
DigitalInOut IO8(GPIO8);
DigitalInOut IO9(GPIO9);
DigitalInOut IO10(GPIO10);
DigitalInOut IO11(GPIO11);
DigitalInOut IO12(GPIO12);
DigitalInOut IO13(GPIO13);
DigitalInOut IO14(GPIO14);
DigitalInOut IO15(GPIO15);
DigitalInOut IO16(GPIO16);
DigitalInOut IO17(GPIO17);
DigitalInOut IO18(GPIO18);
DigitalInOut IO19(GPIO19);
DigitalInOut IO20(GPIO20);
DigitalInOut IO21(GPIO21);
DigitalInOut IO22(GPIO22);
DigitalInOut IO23(GPIO23);
DigitalInOut IO24(GPIO24);
DigitalInOut IO25(GPIO25);
DigitalInOut IO26(GPIO26);
DigitalInOut IO27(GPIO27);

DigitalInOut* gpios[] = {
    &IO0, &IO1, &IO2, &IO3, &IO4, &IO5, &IO6, &IO7,
    &IO8, &IO9, &IO10,&IO11,&IO12,&IO13,&IO14,&IO15,
    &IO16,&IO17,&IO18,&IO19,&IO20,&IO21,&IO22,&IO23,
    &IO24,&IO25,&IO26,&IO27
};

/* SPI (hardware, optional — uses GP9/GP10/GP11) */
static SPI* spi = NULL;
static uint8_t spi_out;

/* Latched pin for auto-increment mode */
static uint8_t latched_pin = 0;

/* State machine for multi-byte commands (SPI) */
static void (*state_handler)(char dat);
static void state_default(char dat);
static void state_spi_xfer_1(char dat);
static void state_spi_xfer_2(char dat);

/* ------ Helpers ------ */

static uint8_t hex_to_nibble(char x) {
    if (x >= '0' && x <= '9') return x - '0';
    if (x >= 'A' && x <= 'F') return x - 'A' + 10;
    if (x >= 'a' && x <= 'f') return x - 'a' + 10;
    return 0;
}

static char nibble_to_hex(uint8_t x) {
    return (x >= 10) ? ('A' + (x - 10)) : ('0' + x);
}

static void inc_latched_pin(void) {
    latched_pin++;
    if (latched_pin >= PIN_COUNT) {
        latched_pin = 0;
    }
}

static void spi_dispose(void) {
    if (spi) {
        delete spi;
        spi = NULL;
    }
}

static SPI* spi_get(void) {
    if (spi) return spi;
    /* GP11=SCK(PA_5), GP10=MOSI(PA_7), GP9=MISO(PA_6) */
    spi = new SPI(GPIO10, GPIO9, GPIO11);
    return spi;
}

/* ------ GPIO command dispatch ------ */

static void gpio_cmd(uint8_t pin, char cmd) {
    if (pin >= PIN_COUNT) return;

    DigitalInOut* io = gpios[pin];

    switch (cmd) {
        case 'I':
            io->input();
            io->mode(PullNone);
            break;
        case 'O':
            io->output();
            break;
        case '0':
            *io = 0;
            break;
        case '1':
            *io = 1;
            break;
        case '?': {
            int p = io->read();
            serialPort.putc('a' + pin);
            serialPort.putc(p ? '1' : '0');
            serialPort.putc('\r');
            serialPort.putc('\n');
            break;
        }
        case 'U':
            io->mode(PullUp);
            break;
        case 'D':
            io->mode(PullDown);
            break;
        case 'N':
            io->mode(PullNone);
            break;
    }
}

/* ------ SPI state machine ------ */

static void state_spi_xfer_2(char dat) {
    SPI* s = spi_get();
    if (s) {
        spi_out |= hex_to_nibble(dat) & 0x0F;
        uint8_t recv = s->write(spi_out);
        serialPort.putc(nibble_to_hex((recv >> 4) & 0xF));
        serialPort.putc(nibble_to_hex(recv & 0xF));
    }
    state_handler = state_default;
}

static void state_spi_xfer_1(char dat) {
    spi_out = hex_to_nibble(dat) << 4;
    state_handler = state_spi_xfer_2;
}

/* ------ Default state: parse pin + command ------ */

static void state_default(char dat) {
    /* Pin latch: 'a'..'a'+27 */
    if (dat >= 'a' && dat < ('a' + PIN_COUNT)) {
        latched_pin = dat - 'a';
        return;
    }

    /* Pin commands */
    if (dat == '0' || dat == '1' ||
        dat == 'I' || dat == 'O' || dat == '?' ||
        dat == 'U' || dat == 'D' || dat == 'N') {
        gpio_cmd(latched_pin, dat);
        inc_latched_pin();
        return;
    }

    /* SPI transfer */
    if (dat == '~') {
        state_handler = state_spi_xfer_1;
        return;
    }
}

/* ------ Global commands (always handled) ------ */

static bool global_handler(char dat) {
    if (dat == 'V') {
        /* Version — reset state to prevent stale SPI state */
        state_handler = state_default;
        serialPort.printf("%s\r\n", VERSION_STR);
        return true;
    }
    if (dat == 'R') {
        /* Reset: set all pins to input, reset latched pin */
        latched_pin = 0;
        for (int i = 0; i < PIN_COUNT; i++) {
            gpios[i]->input();
            gpios[i]->mode(PullNone);
        }
        spi_dispose();
        state_handler = state_default;
        serialPort.printf("OK");
        return true;
    }
    return false;
}

/* ------ Main ------ */

int main() {
    state_handler = state_default;

    serialPort.baud(BAUD_RATE);
    serialPort.format(8, mbed::SerialBase::None, 1);
    serialPort.printf("RTk.GPIO v2.1 Ready\r\n");

    while (1) {
        if (serialPort.readable()) {
            char dat = (char)serialPort.getc();

            /* Ignore \r \n */
            if (dat == '\r' || dat == '\n') continue;

            /* Global commands first */
            if (global_handler(dat)) continue;

            /* State machine */
            if (state_handler) {
                state_handler(dat);
            }
        }
    }
}
