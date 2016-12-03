#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#include "../lib/andygock_avr-uart/uart.h"

#ifndef DEBUG
#define DEBUG 0
#endif
#define debug_print(...) \
            do { if (DEBUG+0) fprintf(stderr, ##__VA_ARGS__); } while (0)

#ifndef F_CPU
#define F_CPU 16000000UL
#endif


// debug leds
#ifndef __AVR_ATmega328__
#define RECV_OK_ERR_SETUP DDRA |= _BV(DDA1) | _BV(DDA0)
#define RECV_OK_ON PORTA |= _BV(PORTA0)
#define RECV_OK_OFF PORTA &= ~_BV(PORTA0)
#define RECV_OK_TOGGLE PORTA ^= _BV(PORTA0)
#define RECV_ERR_ON PORTA |= _BV(PORTA1)
#define RECV_ERR_OFF PORTA &= ~_BV(PORTA1)
#else
#define RECV_OK_ERR_SETUP
#define RECV_OK_ON
#define RECV_OK_OFF
#define RECV_OK_TOGGLE
#define RECV_ERR_ON
#define RECV_ERR_OFF
#endif

void uart0_init(void) {
#ifndef BAUD
#define BAUD 9600
#endif
#include <util/setbaud.h>

    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

static inline void init_pwm() {
    // arduino pins 9(OC1A) and 10(OC1B)
    TCCR1A |= _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
    TCCR1B |= _BV(WGM12) | _BV(CS11) | _BV(CS10);  // divide clock by 64

    // arduino pin 11(OC2A)
    TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM22);
    TCCR2B |= _BV(CS22); // divide clock by 64

    // set pins to output
    DDRB |= _BV(DDB1) | _BV(DDB2) | _BV(DDB3); // 9, 10, 11
}

static inline void init_io() {
    DDRB |= _BV(DDB5) | _BV(DDB4); // pins 13, 12
    DDRD |= _BV(DDD7) | _BV(DDD6) | _BV(DDD5) | _BV(DDD4) | _BV(DDD3) | _BV(DDD1); // pins 8, 7, 5, 4, 3
}

static void set_motor1(uint8_t speed, bool direction, bool enable) {
    debug_print("m1, %d, %d, %d\n", speed, direction, enable);

    if (direction) {
        PORTB |= _BV(PORTB0);
    } else {
        PORTB &= ~_BV(PORTB0); // 8
    }
    if (enable) {
        PORTD |= _BV(PORTD5);
    } else {
        PORTD &= ~_BV(PORTD5); // 5
    }

    if (speed == 0) {
        TCCR2A &= ~_BV(COM2A1);
    } else {
        TCCR2A |= _BV(COM2A1);
        OCR2A = speed; // 11
    }
}

static void set_motor2(uint8_t speed, bool direction, bool enable) {
    debug_print("m2, %d, %d, %d\n", speed, direction, enable);

    if (direction) {
        PORTD |= _BV(PORTD7);
    } else {
        PORTD &= ~_BV(PORTD7); // 7
    }
    if (enable) {
        PORTD |= _BV(PORTD4);
    } else {
        PORTD &= ~_BV(PORTD4); // 4
    }

    if (speed == 0) {
        TCCR1A &= ~_BV(COM1A1);
    } else {
        TCCR1A |= _BV(COM1A1);
        OCR1AH = speed; // 10
    }
}

static void set_motor3(uint8_t speed, bool direction, bool enable) {
    debug_print("m3, %d, %d, %d\n", speed, direction, enable);

    if (direction) {
        PORTD |= _BV(PORTD6);
    } else {
        PORTD &= ~_BV(PORTD6); // 6
    }
    if (enable) {
        PORTD |= _BV(PORTD3);
    } else {
        PORTD &= ~_BV(PORTD3); // 3
    }

    if (speed == 0) {
        TCCR1A &= ~_BV(COM1B1);
    } else {
        TCCR1A |= _BV(COM1B1);
        OCR1BH = speed; // 9
    }
}


static inline void motor_reset() {
    set_motor1(255, false, false);
    set_motor2(255, false, false);
    set_motor3(255, false, false);
}


static inline void set_out(bool kicker, bool grabber) {
    (void) kicker;
    (void) grabber;
}


int uart0_putc(char c, FILE *stream);
int uart0_putc(char c, FILE *stream)
{
    (void) stream;

    if (c == '\n') {
        uart0_putc('\r', stream);
    }

    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

FILE uart0_out = FDEV_SETUP_STREAM(uart0_putc, NULL, _FDEV_SETUP_WRITE);

static inline uint8_t read_uart(uint8_t byte_count, uint8_t packet[], uint8_t packet_len) {
    static uint8_t checksum;
    if (bit_is_set(UCSR0A, RXC0)) {
        uint8_t data = UDR0;
        debug_print("byte_count %d\n", byte_count);

        // find start of a packet with two 0xAA bytes
        if (data == 0xAA && byte_count == 0) {
                debug_print("first start\n");
                return 1;
        } else if (data == 0xAA && byte_count == 1) {
                debug_print("second start\n");
                return 2;
        }

        if (byte_count > 1 && byte_count < packet_len +2) {
            packet[byte_count-2] = data;
            debug_print("data %u, %#02x\n", byte_count-2, data);
            checksum += data;
            return byte_count + 1;
        } else if (byte_count == packet_len +2){
            if (checksum == data) {
                debug_print("checksum OK %#02x\n", checksum);
                RECV_ERR_OFF;
                checksum = 0;
                return byte_count + 1;
            } else {
                debug_print("checksum ERR %#02x, %#02x\n", checksum, data);
                RECV_ERR_ON;
                checksum = 0;
                return 0;
            }
        } else {
            debug_print("packet ERR\n");
            RECV_ERR_ON;
            checksum = 0;
            return 0;
        }
    }
    return byte_count;
}

int main (void)
{
    RECV_OK_ERR_SETUP;
    RECV_OK_ON;
    RECV_ERR_OFF;
    init_io();
    init_pwm();
    uart0_init();
    stderr = stdout = stdin = &uart0_out;
    fprintf(stderr, "Start\n");
    debug_print("Debug mode\n");
    motor_reset();
    uint8_t byte_count = 0;
    uint8_t packet[4] = {0};
    uint8_t packet_len = sizeof(packet) / sizeof(packet[0]);
    //uint32_t counter = 0;
    while (1) {
        byte_count = read_uart(byte_count, packet, packet_len);
        if (byte_count == packet_len + 3) {
            RECV_OK_TOGGLE;
            debug_print("packet received\n");
            set_motor1(packet[0], bit_is_set(packet[3], _BV(0)),
                                  bit_is_set(packet[3], _BV(1)));
            set_motor1(packet[1], bit_is_set(packet[3], _BV(2)),
                                  bit_is_set(packet[3], _BV(3)));
            set_motor1(packet[2], bit_is_set(packet[3], _BV(4)),
                                  bit_is_set(packet[3], _BV(5)));

            byte_count = 0;
            //counter = 0;
        }
        //counter++;
        //if (counter > 1000000) {
        //    debug_print("reset\n");
        //    motor_reset();
        //    counter = 0;
        //}

    }

}
