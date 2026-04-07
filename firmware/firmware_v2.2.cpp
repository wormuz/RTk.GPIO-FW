/*
 * RTk.GPIO Microcontroller Firmware v2.2
 * Based on original v1 by Ryanteck LTD (C) 2016, GNU GPL V3
 * v2 protocol features from bit-hack/RTk.GPIO (C) 2022
 * v2.1 merged + bug fixes 2026-04-07
 * v2.2 PWM support 2026-04-07
 *
 * Protocol:
 *   <pin>I       — input mode
 *   <pin>O       — output mode
 *   <pin>0/1     — write LOW/HIGH
 *   <pin>?       — read → pin+val+\r\n
 *   <pin>U/D/N   — pull up/down/none
 *   <pin>W<4hex> — PWM at frequency (hex Hz), 50% duty
 *   <pin>P<2hex> — PWM set duty (00-FF → 0-100%)
 *   <pin>X       — stop PWM, return to GPIO
 *   ~<2hex>      — SPI transfer byte → 2hex response
 *   V            — version string
 *   R            — reset all → "OK"
 *
 * PWM-capable pins (STM32F030C8T6):
 *   GP4  (PA8)  = TIM1_CH1
 *   GP9  (PA6)  = TIM3_CH1
 *   GP10 (PA7)  = TIM3_CH2
 *   GP16 (PB8)  = TIM16_CH1
 *   GP18 (PB1)  = TIM3_CH4
 *   GP20 (PB9)  = TIM17_CH1
 *   GP26 (PB0)  = TIM3_CH3
 *
 * Target: STM32F030C8T6 (48-pin, 12MHz HSE crystal)
 * Board: RTk.GPIO + ReSpeaker 2-Mics HAT
 * mbed: v1 mbed-dev (HSI fallback, proven stable)
 */

#include "mbed.h"

#define BAUD_RATE   230400
#define PIN_COUNT   28
#define VERSION_STR "RTk.GPIO v2.2 2026-04-07"

/* UART on PA_9 (TX) / PA_10 (RX) — connected to CH340G */
Serial serialPort(SERIAL_TX, SERIAL_RX);

/* Pin name table (same order as GPIOx defines) */
static const PinName pinNames[] = {
    GPIO0,  GPIO1,  GPIO2,  GPIO3,  GPIO4,  GPIO5,  GPIO6,  GPIO7,
    GPIO8,  GPIO9,  GPIO10, GPIO11, GPIO12, GPIO13, GPIO14, GPIO15,
    GPIO16, GPIO17, GPIO18, GPIO19, GPIO20, GPIO21, GPIO22, GPIO23,
    GPIO24, GPIO25, GPIO26, GPIO27
};

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

/* PWM objects (dynamically created, NULL when not active) */
static PwmOut* pwm_out[PIN_COUNT] = {NULL};

/* SPI (hardware, optional — uses GP9/GP10/GP11) */
static SPI* spi = NULL;
static uint8_t spi_out;

/* Latched pin for auto-increment mode */
static uint8_t latched_pin = 0;

/* State machine */
static void (*state_handler)(char dat);
static void state_default(char dat);
static void state_spi_xfer_1(char dat);
static void state_spi_xfer_2(char dat);
static void state_pwm_freq_1(char dat);
static void state_pwm_freq_2(char dat);
static void state_pwm_freq_3(char dat);
static void state_pwm_freq_4(char dat);
static void state_pwm_duty_1(char dat);
static void state_pwm_duty_2(char dat);

/* PWM command state */
static uint8_t pwm_cmd_pin;
static uint16_t pwm_cmd_freq;
static uint8_t pwm_cmd_duty;

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

/* ------ PWM management ------ */

static void pwm_stop(uint8_t pin) {
    if (pin >= PIN_COUNT) return;
    if (pwm_out[pin]) {
        delete pwm_out[pin];
        pwm_out[pin] = NULL;
        /* Re-init as GPIO input */
        gpios[pin]->input();
        gpios[pin]->mode(PullNone);
    }
}

static bool pwm_start(uint8_t pin, uint16_t freq_hz) {
    if (pin >= PIN_COUNT || freq_hz == 0) return false;

    /* Stop existing PWM on this pin */
    pwm_stop(pin);

    /* Create PwmOut — mbed will pick the right timer */
    pwm_out[pin] = new PwmOut(pinNames[pin]);
    if (!pwm_out[pin]) return false;

    /* Set period from frequency */
    float period = 1.0f / (float)freq_hz;
    pwm_out[pin]->period(period);
    pwm_out[pin]->write(0.5f);  /* 50% duty default */

    return true;
}

static void pwm_set_duty(uint8_t pin, uint8_t duty_byte) {
    if (pin >= PIN_COUNT || !pwm_out[pin]) return;
    /* duty_byte 0x00=0%, 0xFF=100% */
    float duty = (float)duty_byte / 255.0f;
    pwm_out[pin]->write(duty);
}

/* ------ SPI management ------ */

static void spi_dispose(void) {
    if (spi) {
        delete spi;
        spi = NULL;
    }
}

static SPI* spi_get(void) {
    if (spi) return spi;
    spi = new SPI(GPIO10, GPIO9, GPIO11);
    return spi;
}

/* ------ GPIO command dispatch ------ */

static void gpio_cmd(uint8_t pin, char cmd) {
    if (pin >= PIN_COUNT) return;

    /* If PWM active on this pin and non-PWM command, stop PWM first */
    if (pwm_out[pin] && cmd != 'W' && cmd != 'P' && cmd != 'X') {
        pwm_stop(pin);
    }

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

/* ------ PWM frequency state machine (4 hex chars = 16-bit Hz) ------ */

static void state_pwm_freq_4(char dat) {
    pwm_cmd_freq |= hex_to_nibble(dat);
    bool ok = pwm_start(pwm_cmd_pin, pwm_cmd_freq);
    serialPort.putc(ok ? 'K' : 'E');
    state_handler = state_default;
}

static void state_pwm_freq_3(char dat) {
    pwm_cmd_freq |= hex_to_nibble(dat) << 4;
    state_handler = state_pwm_freq_4;
}

static void state_pwm_freq_2(char dat) {
    pwm_cmd_freq = hex_to_nibble(dat) << 8;
    state_handler = state_pwm_freq_3;
}

static void state_pwm_freq_1(char dat) {
    pwm_cmd_freq = (uint16_t)hex_to_nibble(dat) << 12;
    pwm_cmd_pin = latched_pin;
    state_handler = state_pwm_freq_2;
}

/* ------ PWM duty state machine (2 hex chars = 8-bit duty) ------ */

static void state_pwm_duty_2(char dat) {
    pwm_cmd_duty |= hex_to_nibble(dat);
    pwm_set_duty(pwm_cmd_pin, pwm_cmd_duty);
    state_handler = state_default;
}

static void state_pwm_duty_1(char dat) {
    pwm_cmd_duty = hex_to_nibble(dat) << 4;
    pwm_cmd_pin = latched_pin;
    state_handler = state_pwm_duty_2;
}

/* ------ Default state: parse pin + command ------ */

static void state_default(char dat) {
    /* Pin latch: 'a'..'a'+27 */
    if (dat >= 'a' && dat < ('a' + PIN_COUNT)) {
        latched_pin = dat - 'a';
        return;
    }

    /* GPIO commands */
    if (dat == '0' || dat == '1' ||
        dat == 'I' || dat == 'O' || dat == '?' ||
        dat == 'U' || dat == 'D' || dat == 'N') {
        gpio_cmd(latched_pin, dat);
        inc_latched_pin();
        return;
    }

    /* PWM start: W + 4 hex chars (frequency in Hz) */
    if (dat == 'W') {
        state_handler = state_pwm_freq_1;
        return;
    }

    /* PWM duty: P + 2 hex chars (0x00-0xFF) */
    if (dat == 'P') {
        state_handler = state_pwm_duty_1;
        return;
    }

    /* PWM stop: X */
    if (dat == 'X') {
        pwm_stop(latched_pin);
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
        state_handler = state_default;
        serialPort.printf("%s\r\n", VERSION_STR);
        return true;
    }
    if (dat == 'R') {
        /* Full reset: GPIO, PWM, SPI */
        latched_pin = 0;
        for (int i = 0; i < PIN_COUNT; i++) {
            pwm_stop(i);
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
    serialPort.printf("RTk.GPIO v2.2 Ready\r\n");

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
