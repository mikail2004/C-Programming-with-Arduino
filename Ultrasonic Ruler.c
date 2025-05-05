/************************************ 
File Name: main.c
Author: Mikail Usman
Date: 4/28/2025
Description: Measure distances using Ultrasound sensor and display on 4-digit LED
************************************/

#include "my_adc_lib.h"
#include "my_uart_lib.h"
#include "SSD1306.h"
#include "i2c.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#define TRIG PC1
#define ECHO PC0
#define clockRange 1.098
#define delaySET 2

unsigned char ledDigits[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x67};

void timer0_init(void);
unsigned char measure_distance_cm(void);
void display_number(unsigned int number);

int main(void) {
    unsigned int distance_cm;

    DDRC |= (1<<TRIG);  // Setting TRIG, ECHO for Ultrasound sensor (Analog ports)
    DDRD = 0xFF; // PD for display segments requiring resistor 
    DDRB |= (1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB1); // PB for display segments requiring no resistor
    PORTB |= (1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB1); // All digits start turned off

    timer0_init();

    while (1) {
        distance_cm = measure_distance_cm();

        for (int i = 0; i < 100; i++) { // Setting refresh rate for the display
            if (distance_cm >=5 && distance_cm <=200) {
                display_number(distance_cm);
            } else {
                display_number(0); // Display 0000 if out of sensor range
            }
        }
    }
}

// Measuring sensor distance in cm
unsigned char measure_distance_cm(void) {
    unsigned char rising_edge_clocks, falling_edge_clocks, echo_width_clocks;
    float target_range;

    TCNT0 = 0;
    PORTC |= (1<<TRIG);
    _delay_us(10);
    PORTC &= ~(1<<TRIG);

    while ((PINC & (1<<ECHO)) == 0);
    rising_edge_clocks = TCNT0;

    while ((PINC & (1<<ECHO)) != 0);
    falling_edge_clocks = TCNT0;

    if (falling_edge_clocks > rising_edge_clocks) {
        echo_width_clocks = falling_edge_clocks - rising_edge_clocks;
        target_range = echo_width_clocks * clockRange;
        return (unsigned char)(target_range + 0.5); // Rounding to nearest cm
    }
    else {
        return 0; // Error handling if reading issue
    }
}

// Displaying a 4-digit number from sensor values (CM Reading)
void display_number(unsigned int number) {
    unsigned char DIG1, DIG2, DIG3, DIG4; // 1, 2, 3, 4 (From left to right)

    DIG4 = number % 10;
    DIG3 = (number/10)%10;
    DIG2 = (number/100)%10;
    DIG1 = (number/1000)%10;

    // Digit 4 
    PORTD = ledDigits[DIG4];
    PORTB = ~(1<<PB2);
    _delay_ms(delaySET);
    PORTB |= (1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB1);

    // Digit 3
    PORTD = ledDigits[DIG3];
    PORTB = ~(1<<PB3);
    _delay_ms(delaySET);
    PORTB |= (1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB1);

    // Digit 2
    PORTD = ledDigits[DIG2];
    PORTB = ~(1<<PB4);
    _delay_ms(delaySET);
    PORTB |= (1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB1);

    // Digit 1
    PORTD = ledDigits[DIG1];
    PORTB = ~(1<<PB1);
    _delay_ms(delaySET);
    PORTB |= (1<<PB2)|(1<<PB3)|(1<<PB4)|(1<<PB1);
}

void timer0_init() {
    TCCR0A = 0;
    TCCR0B = 5;
    TCNT0 = 0;
}
