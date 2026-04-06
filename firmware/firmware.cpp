#include "mbed.h"

#define BAUD_RATE   230400
#define PIN_COUNT   28
#define VERSION_STR "RTk.GPIO v2.1 2026-04-07\n"
#define READY_STR   "RTk.GPIO v2.1 Ready\n"

// pin number mapping
static const PinName gpPinMap[] = {
    PA_1 , PB_12, PB_7 , PB_6 , PA_8 , PA_12, PA_13, PF_1 ,
    PB_5 , PA_6 , PA_7 , PA_5 , PF_0 , PA_14, PA_2 , PA_3 ,
    PB_8 , PB_15, PB_1 , PA_15, PB_9 , PB_10, PA_11, PB_2 ,
    PB_3 , PB_4 , PB_0 , PB_14,
};
static DigitalInOut *gp[PIN_COUNT];

// SPI bus
static const PinName spiPinSck  = PA_5;  // gp11
static const PinName spiPinMiso = PA_6;  // gp9
static const PinName spiPinMosi = PA_7;  // gp10
static SPI *spi;
// data to be transmited from the spi interface
static uint8_t spi_out;

// UART serial port
static Serial serialPort(/*TX=*/PA_9, /*RX=*/PA_10);

// index of the next pin operation
static uint8_t latched_pin = 0;

// currently bound state handler
static void (*state_handler)(const char dat);

// state machine state handlers
static void state_spi_xfer_1  (const char dat);
static void state_spi_xfer_2  (const char dat);
static void state_default(const char dat);

// dispose of a bound gpio object
static void gpio_dispose(uint8_t pin) {
    if (gp[pin]) {
        delete gp[pin];
        gp[pin] = NULL;
    }
}

// dispose of a bound spi object
static void spi_dispose(void) {
    if (spi) {
        delete spi;
        spi = NULL;
    }
}

// access a pin as a GPIO interface
static DigitalInOut *gpio_get(uint8_t pin) {
    // check if the digital pin already exists
    if (gp[pin]) {
        return gp[pin];
    }
    // get the mbed pin
    const PinName mpin = gpPinMap[pin];
    // check if we conflict with the spi object
    const bool uses_spi = (mpin == spiPinMiso) ||
                          (mpin == spiPinMosi) ||
                          (mpin == spiPinSck);
    if (uses_spi) {
        spi_dispose();
    }
    // create the new GPIO object
    return gp[pin] = new DigitalInOut(mpin);
}

// access the SPI bus
static SPI *spi_get() {
    // check if spi object already exists
    if (spi) {
        return spi;
    }
    // check if one of the SPI pins is currently used
    gpio_dispose(9);   // spiPinMiso
    gpio_dispose(10);  // spiPinMosi
    gpio_dispose(11);  // spiPinSck
    // create the new SPI object
    return spi = new SPI(spiPinMosi, spiPinMiso, spiPinSck);
}

// convert hex chars to a binary nibble
static uint8_t hex_to_nibble(char x) {
    return (x >= '0' && x <= '9') ? (x - '0') : ((x - 'A') + 10);
}

// convert a binary nibble to hex chars
static char nibble_to_hex(uint8_t x) {
    return (x >= 10) ? ('A' + (x - 10)) : ('0' + x);
}

// perform a pin related action
static void dispatch_pin(uint8_t pin, uint8_t action) {
    if (pin >= PIN_COUNT) {
        return;
    }
    // fetch the IO object for this pin or create on demand
    DigitalInOut *io = gpio_get(pin);
    if (io) {
        // dispatch the operation
        switch (action) {
        case '0': io->write(0);       break;
        case '1': io->write(1);       break;
        case 'O': io->output();       break;
        case 'D': io->mode(PullDown); break;
        case 'U': io->mode(PullUp);   break;
        case 'N': io->mode(PullNone); break;
        case 'I':
            io->input();
            break;
        case '?':
            serialPort.putc('a' + pin);
            serialPort.putc( io->read() ? '1' : '0' );
            serialPort.putc('\r');
            serialPort.putc('\n');
            break;
        }
    }
}

// increment and wrap the latched pin index
static void inc_latched_pin(void) {
    ++latched_pin;
    if (latched_pin >= PIN_COUNT) {
        latched_pin = 0;
    }
}

// reset all assumed state
static void reset() {
    // reset the latched pin
    latched_pin = 0;
    // dispose of all GPIO pins
    for (int i=0; i<PIN_COUNT; ++i) {
        gpio_dispose(i);
    }
    // dispose of the SPI interface
    spi_dispose();
    // default to root state
    state_handler = state_default;
}

// SPI transmit state 2
// latch second byte and then perform the transfer operation
static void state_spi_xfer_2(const char dat) {
    // get the spi object we need
    SPI *spi = spi_get();
    if (spi) {
        // latch spi least significant nibble
        const uint8_t d1 = hex_to_nibble(dat);
        spi_out |= d1 & 0x0f;
        // send byte over spi
        const uint8_t recv = spi->write(spi_out);
        // send response back to host
        serialPort.putc(nibble_to_hex((recv >> 4) & 0xf));  // msb
        serialPort.putc(nibble_to_hex((recv     ) & 0xf));  // lsb
    }
    // set next state
    state_handler = state_default;
}

// SPI transmit state 1
// latch first byte then wait for the second
static void state_spi_xfer_1(const char dat) {
    // latch spi most significant nibble
    const uint8_t d0 = hex_to_nibble(dat);
    spi_out = d0 << 4;
    // set next state
    state_handler = state_spi_xfer_2;
}

// default state
static void state_default(const char dat) {
    // pin adjustment
    if (dat >= 'a' && dat < ('a' + PIN_COUNT)) {
        // latch this pin for later operations
        latched_pin = uint8_t(dat - 'a');
        return;
    }
    // latched pin adjustment
    if (dat == '0' || dat == '1' ||
        dat == 'I' || dat == 'O' ||
        dat == '?' ||
        dat == 'U' || dat == 'D' || dat == 'N') {
        // perform this action
        dispatch_pin(/*pin=*/latched_pin, /*op=*/dat);
        // increment the latched pin
        inc_latched_pin();
        return;
    }
    // SPI operation
    if (dat == '~') {
        // enter the spi transfer state
        state_handler = state_spi_xfer_1;
        return;
    }
}

static bool global_handler(const char dat) {
    // return version string
    if (dat == 'V') {
        // FIX: reset state machine to prevent stale SPI state
        state_handler = state_default;
        serialPort.puts(VERSION_STR);
        return true;
    }
    // reset state
    if (dat == 'R') {
        reset();
        // send ack
        serialPort.puts("OK");
        return true;
    }
    return false;
}

int main() {
    // reset the GPIO board state
    reset();
    // setup the serial port
    serialPort.baud(BAUD_RATE);
    serialPort.format(
        /*     bits=*/8,
        /*   parity=*/mbed::SerialBase::None,
        /*stop_bits=*/1);
    serialPort.printf(READY_STR);
    // main loop
    for (;;) {
        // wait for data to be available
        if (!serialPort.readable()) {
            continue;
        }
        // note that in this design this is the only place that reads from
        // the serial port.  this is important to avoid lockups when waiting
        // for data in nested code.  by reading in one place we can support
        // a global handler that can perform resets consistently.
        const uint8_t dat = serialPort.getc();
        // allow a global handler to deal with this first
        if (global_handler(dat)) {
            continue;
        }
        // invoke state handler
        if (state_handler) {
            state_handler(dat);
        }
    }
}
