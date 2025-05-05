/************************************ 
File Name: main.c
Author: Mikail Usman
Date: 4/25/2025
Description: Measure temperature and display readings on Laptop display and OLED.
************************************/

#include "my_adc_lib.h"
#include "my_uart_lib.h"
#include "SSD1306.h"
#include "i2c.h"
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

// Defining variables to pins (Using 'Digital PWM' Pins PD7-PD2)
#define BUTTON PD6 // Assigned to button for toggling temperature unit (Pin 6)
#define REDLED  PD7 // Assigned to Red LED (Pin 7)
#define tempThreshold 80.0 // Temperature threshold

// Converting ADC values to temperature in Celsius
float get_temperature_celsius(unsigned int adc_value) {
    float voltage = adc_value * 5.0 / 1023.0; // Convert ADC to voltage
    float tempC = (voltage - 0.5) * 100.0;
    return tempC;
}

// Displaying temperature readings (OLED, PUTTY)
void display_temperature(float temp, char unit) {
    char buffer[10];
    dtostrf(temp, 5, 1, buffer); // Converting float to string (1 decimal precision)
    char full_string[12];
    snprintf(full_string, sizeof(full_string), "%s%c", buffer, unit); // Formatting reading

    // Displaying on OLED
    OLED_GoToLine(2);
    OLED_DisplayString("Temp: ");
    OLED_DisplayString(full_string);

    // Displaying on Laptop Display using PUTTY (UART)
    send_string("Temp: ");
    send_string(full_string);
    send_string("\n");
}

int main(void) {
    DDRD &= ~(1 << BUTTON); // Set PD6 as input
    PORTD |= (1 << BUTTON); // Enable pull-up on PD6
    DDRD |= (1 << REDLED);  // Set PD7 as output (Red LED)

    OLED_Init();
    adc_init();
    uart_init();

    unsigned int adc_value;
    float tempC, tempF;

    while (1) {
        adc_value = get_adc(); // Reading ADC value
        tempC = get_temperature_celsius(adc_value);
        tempF = (tempC * 9.0 / 5.0) + 32.0; // Temperature reading in Fahrenheit

        // Checking button state (Celsius if pressed)
        if ((PIND & (1 << BUTTON)) == 0) {
            display_temperature(tempC, 'C');
        } else {
            display_temperature(tempF, 'F');
        }

        // Turn on LED if temp in Fahrenheit > tempThreshold
        if (tempF > tempThreshold) {
            PORTD |= (1 << REDLED);
        } else {
            PORTD &= ~(1 << REDLED);
        }

        _delay_ms(1000); // 1 second update rate
    }
}