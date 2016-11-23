#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#include "../lib/andygock_avr-uart/uart.h"

#ifndef DEBUG
#define DEBUG 0
#endif
#define debug_print(...) \
            do { if (DEBUG) fprintf(stderr, ##__VA_ARGS__); } while (0)

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUD
#define BAUD 9600
#endif
#include <util/setbaud.h>


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

void uart0_init(void)
{
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

#ifndef __AVR_ATmega328__
void uart3_init(void)
{
    UBRR3H = UBRRH_VALUE;
    UBRR3L = UBRRL_VALUE;
#if USE_2X
    UCSR3A |= _BV(U2X3);
#else
    UCSR3A &= ~(_BV(U2X3));
#endif
    UCSR3C = _BV(UCSZ31) | _BV(UCSZ30); /* 8-bit data */
    UCSR3B = _BV(TXEN3);   /* Enable TX */
}
#else
void uart3_init() {
    return;
}
#endif



static inline void init_pwm() {
#ifndef __AVR_ATmega328__
    // arduino pins 9(OC2B) and 10(OC2A)
    TCCR2A |= _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B |= _BV(CS22);  // divide clock by 64

    // arduino pin 11(OC1A)
    TCCR1A |= _BV(COM1A1) | _BV(WGM10);
    TCCR1B |= _BV(CS10) | _BV(CS11) | _BV(WGM12); // divide clock by 64

    // set pins to output
    DDRB |= _BV(DDB5); // 11
    DDRB |= _BV(DDB4); // 10
    DDRH |= _BV(DDH6); // 9
#endif
}

static inline void init_io() {
#ifndef __AVR_ATmega328__
    DDRH |= _BV(DDH5) | _BV(DDH4) | _BV(DDH3); // pins 8 7 6
    DDRE |= _BV(DDE3) | _BV(DDE5); // pins 5 3
    DDRG |= _BV(DDG5); // pin 4
#endif
}

static void set_motor1(uint8_t pwm, bool fwd, bool rev) {
    debug_print("m1, %d, %d, %d\n", pwm, fwd, rev);
#ifndef __AVR_ATmega328__
    TCCR1A |= _BV(COM1A1);
    OCR1A = 255;

    if (fwd) {
        PORTH |= _BV(PORTH5);
    } else {
        PORTH &= ~_BV(PORTH5); // 8
    }
    if (rev) {
        PORTE |= _BV(PORTE3);
    } else {
        PORTE &= ~_BV(PORTE3); // 5
    }

    if (pwm == 0) {
        TCCR1A &= ~_BV(COM1A1);
    } else {
        TCCR1A |= _BV(COM1A1);
        OCR1A = pwm; // 11
    }
#endif
}

static void set_motor2(uint8_t pwm, bool fwd, bool rev) {
    debug_print("m2, %d, %d, %d\n", pwm, fwd, rev);
#ifndef __AVR_ATmega328__
    TCCR2A |= _BV(COM2A1);
    OCR2A = 255;

    if (fwd) {
        PORTH |= _BV(PORTH4);
    } else {
        PORTH &= ~_BV(PORTH4); // 7
    }
    if (rev) {
        PORTG |= _BV(PORTG5);
    } else {
        PORTG &= ~_BV(PORTG5); // 4
    }

    if (pwm == 0) {
        TCCR2A &= ~_BV(COM2A1);
    } else {
        TCCR2A |= _BV(COM2A1);
        OCR2A = pwm; // 9
    }
#endif
}

static void set_motor3(uint8_t pwm, bool fwd, bool rev) {
    debug_print("m3, %d, %d, %d\n", pwm, fwd, rev);
#ifndef __AVR_ATmega328__
    TCCR2A |= _BV(COM2B1);
    OCR2B = 255;

    if (fwd) {
        PORTH |= _BV(PORTH3);
    } else {
        PORTH &= ~_BV(PORTH3); // 7
    }
    if (rev) {
        PORTE |= _BV(PORTE5);
    } else {
        PORTE &= ~_BV(PORTE5); // 3
    }

    if (pwm == 0) {
        TCCR2A &= ~_BV(COM2B1);
    } else {
        TCCR2A |= _BV(COM2B1);
        OCR2B = pwm; // 9
    }
#endif
}

static uint8_t speed2pwm(int8_t speed) {
    debug_print("speed2pwm speed: %d, abs: %d, ", speed, abs(speed));
    uint8_t pwm = 0;
    if (speed < 0) {
        debug_print("less 0, ");
        pwm = (abs(speed)-1)<<1;
        pwm++;
    } else if (speed > 0){
        debug_print("more 0, ");
        pwm = (speed<<1)+1;
    } else {
        debug_print("is 0, ");
        pwm = 0;
    }
    pwm = 255 - pwm;
    debug_print("pwm: %u\n", pwm);
    return pwm;
}
static inline void motor_reset() {
    set_motor1(255, false, false);
    set_motor2(255, false, false);
    set_motor3(255, false, false);
}

static void set_speed(int8_t m1, int8_t m2, int8_t m3) {

    // reset
    motor_reset();

    set_motor1(speed2pwm(m1), m1 <= 0 ? 1:0, m1 >= 0 ? 1:0);
    set_motor2(speed2pwm(m2), m2 <= 0 ? 1:0, m2 >= 0 ? 1:0);
    set_motor3(speed2pwm(m3), m3 <= 0 ? 1:0, m3 >= 0 ? 1:0);
}


int uart0_putc(char c, FILE *stream);
int uart0_putc(char c, FILE *stream)
{
    (void) stream;

    //if (c == '\n') {
    //    uart0_putc('\r', stream);
    //}

    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

int uart3_putc(char c, FILE *stream);
int uart3_putc(char c, FILE *stream)
{
    (void) stream;

    if (c == '\n') {
        uart3_putc('\r', stream);
    }
#ifndef __AVR_ATmega328__
    loop_until_bit_is_set(UCSR3A, UDRE3);
    UDR3 = c;
#else
    (void) c;
#endif
    return 0;
}

FILE uart0_out = FDEV_SETUP_STREAM(uart0_putc, NULL, _FDEV_SETUP_WRITE);
FILE uart3_out = FDEV_SETUP_STREAM(uart3_putc, NULL, _FDEV_SETUP_WRITE);

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
    uart3_init();
    stdout = stdin = &uart0_out;
    stderr = &uart3_out;
    fprintf(stderr, "Start\n");
    debug_print("Debug mode\n");
    set_speed(0,0,0);
    uint8_t byte_count = 0;
    uint8_t packet[3] = {0};
    uint8_t packet_len = sizeof(packet) / sizeof(packet[0]);
    uint32_t counter = 0;
    while (1) {
        byte_count = read_uart(byte_count, packet, packet_len);
        if (byte_count == packet_len + 3) {
            RECV_OK_TOGGLE;
            debug_print("packet received\n");
            set_speed(*(uint8_t*)&packet[0], *(uint8_t*)&packet[1], *(uint8_t*)&packet[2]);
            byte_count = 0;
            counter = 0;
        }
        counter++;
        if (counter > 1000000) {
            debug_print("reset\n");
            motor_reset();
            counter = 0;
        }

    }

}
