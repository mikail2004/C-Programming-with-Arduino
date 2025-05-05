/************************************ 
File Name: main.c
Author: Mikail Usman
Date: 4/12/2025
Description: Blinking LEDs with a series of push-buttons.
************************************/

#include <avr/io.h>
#include <util/delay.h>

// Defining variables to pins (Using 'Digital PWM' Pins PD7-PD2)
#define BUTTON1 PD2 // Assigned to button for Red LED (Pin 2)
#define BUTTON2 PD4 // Assigned to button for Green LED (Pin 4)
#define BUTTON3 PD6 // Assigned to button for both LEDs (Pin 6)
#define GREENLED PD3 // Assigned to Green LED (Pin 3)
#define REDLED  PD5 // Assigned to Red LED (Pin 5)

int main(void) {
    DDRD |= (1 << REDLED) | (1 << GREENLED); // Configuring LED pins as outputs
    PORTD |= (1 << BUTTON1) | (1 << BUTTON2) | (1 << BUTTON3); // Configuring button pins as inputs with pull-up resistors

    while (1) {
        // Checking button states (0 = pressed, 1 = not pressed)
        uint8_t b1 = !(PIND & (1 << BUTTON1));
        uint8_t b2 = !(PIND & (1 << BUTTON2));
        uint8_t b3 = !(PIND & (1 << BUTTON3));

        uint8_t pressedCount = b1 + b2 + b3;

        if (pressedCount == 1) {
            // Only one button pressed
            if (b1) {
                // Blink Red LED at 5 Hz
                PORTD |= (1 << REDLED);
                PORTD &= ~(1 << GREENLED);
                _delay_ms(50);
                PORTD &= ~(1 << REDLED);
                _delay_ms(50);
            } else if (b2) {
                // Blink Green LED at 5 Hz
                PORTD |= (1 << GREENLED);
                PORTD &= ~(1 << REDLED);
                _delay_ms(50);
                PORTD &= ~(1 << GREENLED);
                _delay_ms(50);
            } else if (b3) {
                // Blink both LEDs at 5 Hz
                PORTD |= (1 << REDLED) | (1 << GREENLED);
                _delay_ms(50);
                PORTD &= ~(1 << REDLED) & ~(1 << GREENLED);
                _delay_ms(50);
            }
        } else {
            // Turn off LEDs if multiple buttons pressed.
            PORTD &= ~(1 << REDLED);
            PORTD &= ~(1 << GREENLED);
        }
    }
}
