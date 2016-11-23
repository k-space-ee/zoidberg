#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
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

#define RECV_OK_ERR_SETUP DDRA |= _BV(DDA1) | _BV(DDA0)
#define RECV_OK_ON PORTA |= _BV(PORTA0)
#define RECV_OK_OFF PORTA &= ~_BV(PORTA0)
#define RECV_OK_TOGGLE PORTA ^= _BV(PORTA0)
#define RECV_ERR_ON PORTA |= _BV(PORTA1)
#define RECV_ERR_OFF PORTA &= ~_BV(PORTA1)

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

static inline void init_pwm() {
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
}

static inline void set_pwm(uint8_t m1, uint8_t m2, uint8_t m3) {
    //11 = m1
    OCR1AL = m1;
    //9 = m2
    OCR2B = m2;
    //10 = m3
    OCR2A = m3;
}

static inline void init_adc() {

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

    loop_until_bit_is_set(UCSR3A, UDRE3);
    UDR3 = c;
    return 0;
}

FILE uart0_out = FDEV_SETUP_STREAM(uart0_putc, NULL, _FDEV_SETUP_WRITE);
FILE uart3_out = FDEV_SETUP_STREAM(uart3_putc, NULL, _FDEV_SETUP_WRITE);

static inline uint8_t read_uart(uint8_t byte_count) {
    static uint8_t checksum;
    //return byte_count;
    //char bin[10] = {0};
    if (bit_is_set(UCSR0A, RXC0)) {
        RECV_OK_TOGGLE;
        uint8_t data = UDR0;
        (void) data;
        //fprintf(stderr, "recv, %d\n", byte_count);
        //uart0_putc(' ', stdout);
        //itoa(byte_count, bin, 10);
        //puts(bin);
        //uart0_putc(' ');
        //uart0_putc('\r');
        //uart0_putc('\n');
        //uart0_putc(data, stdout);
        //return byte_count;

        // find start of a packet with two 0xAA bytes
        if (data == 0xAA && byte_count == 0) {
                debug_print("first start\n");
                return 1;
        } else if (data == 0xAA && byte_count == 1) {
                debug_print("second start\n");
                return 2;
        }

        //uart0_puts("reading data\n\r");
        switch (byte_count) {
            case 2:
                debug_print("data1, %#02x\n", data);
                OCR1AL = data;
                checksum += data;
                break;
            case 3:
                debug_print("data2, %#02x\n", data);
                OCR2B = data;
                checksum += data;
                break;
            case 4:
                debug_print("data3, %#02x\n", data);
                OCR2A = data;
                checksum += data;
                break;
            case 5:
                debug_print("chksum, %#02x %#02x\n", data, checksum);
                checksum = 0;
                RECV_ERR_OFF;
                RECV_OK_ON;
                return 0; // end of packet
            default:
                RECV_ERR_ON;
                RECV_OK_OFF;
                checksum = 0;
                debug_print("reset\n");
                // something went wrong, end of packet and try to resync
                return 0;
        }
        return ++byte_count;
    } else {
        return byte_count;
    }
}

int main (void)
{
    RECV_OK_ERR_SETUP;
    RECV_OK_ON;
    RECV_ERR_OFF;
    init_pwm();
    uart0_init();
    uart3_init();
    stdout = stdin = &uart0_out;
    stderr = &uart3_out;
    fprintf(stderr, "Start\n");
    debug_print("Debug mode\n");
    //uart0_init(UART_BAUD_SELECT(BAUDRATE, F_CPU));
    //sei();
    //DDRF = 0;
    uint8_t byte_count = 0;
    //(void) byte_count;
    //char bin[10] = {0};
    //puts("Start");
    //_delay_ms(1000);
    while (1) {
        /*
        if(bit_is_set(UCSR0A, RXC0)){
            uint8_t c = UDR0;
            loop_until_bit_is_set(UCSR0A, UDRE0);
            UDR0 = c;
            //putc(c, stderr  );
            //uart0_putc(UDR0, stdout);
        }*/
        //fprintf(stderr, "alive, %d\r", byte_count);
        byte_count = read_uart(byte_count);

        //itoa(byte_count, bin, 2);
        //puts(bin);
        //putc(byte_count, stdout);
        //_delay_ms(500);
        //uart0_putc('\r');
        //uart0_putc('\n');
        //_delay_ms(1000);
        //for (uint8_t i = 0; i < 0xff; i++) {
        //    set_pwm(i, i, i);
        //    _delay_ms(1);
        //}
        //for (uint8_t i = 0xff; i != 0; i--) {
        //    set_pwm(i, i, i);
        //    _delay_ms(1);
        //}
    }

}
