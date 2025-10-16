// main.c - AVR Reflow Plate (pure C version)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include "ssd1306.h"


#define F_CPU 8000000UL

#define MOSFET_PIN PD3
#define BTN_UP     PD6
#define BTN_DOWN   PD5

#define TEMP_ADC   3  // PC3
#define VOLT_ADC   0  // PC0

#define MAX_TEMP_ADDR 1
#define V_CONVERT  4.0f
#define V_MIN      7

// PID tuning parameters (adjust for your plate)
#define KP  0.1f // Proportional gain
#define KI  0.02f  // Integral gain
#define KD  0.1f  // Derivative gain

#define PWM_MAX  190
#define PWM_MIN  0


void adc_init(void);
uint16_t adc_read(uint8_t ch);
int get_temp(void);
int get_volt(void);
void pwm_init(void);
void heat_cycle(uint8_t maxTemp);
uint8_t read_buttons(void);
float pid_update(float setpoint, float current, float dt_ms);


int main(void) {
    DDRD |= (1 << MOSFET_PIN);
    PORTD |= (0 << BTN_UP) | (0 << BTN_DOWN); // pull-ups
    DDRC &= ~(1 << PC2);   // force PC2 as input
    PORTC &= ~(1 << PC2);  // disable pull-up
    pwm_init();
    adc_init();
    SSD1306_Init(0x3C);

    uint8_t maxTemp = eeprom_read_byte((uint8_t*)MAX_TEMP_ADDR);
    if (maxTemp > 4) maxTemp = 2;

    SSD1306_ClearScreen();
    SSD1306_SetPosition(0, 1);
    SSD1306_DrawString("    Reflow Plate");
    SSD1306_UpdateScreen(SSD1306_ADDR);
    _delay_ms(2000);


    while (1) {
        SSD1306_ClearScreen();
        SSD1306_SetPosition(0, 0);
        SSD1306_DrawString("Hold Btns: Start");
        SSD1306_UpdateScreen(SSD1306_ADDR);

        uint8_t b = read_buttons();
        if (b & (1 << BTN_UP)) {
            if (maxTemp < 4) maxTemp++;
            eeprom_write_byte((uint8_t*)MAX_TEMP_ADDR, maxTemp);
        } else if (b & (1 << BTN_DOWN)) {
            if (maxTemp > 0) maxTemp--;
            eeprom_write_byte((uint8_t*)MAX_TEMP_ADDR, maxTemp);
        }

        SSD1306_SetPosition(0, 1);
        char buf[20];
        sprintf(buf, "Max Temp: %d", 140 + 10 * maxTemp);
        SSD1306_DrawString(buf);
        SSD1306_UpdateScreen(SSD1306_ADDR);

        int temp = get_temp();
        SSD1306_SetPosition(0, 2);
        sprintf(buf, "Temp: %d", temp);
        SSD1306_DrawString(buf);
        SSD1306_UpdateScreen(SSD1306_ADDR);

        int volt = get_volt();
        SSD1306_SetPosition(0, 3);
        sprintf(buf, "Volt: %d", volt);
        SSD1306_DrawString(buf);
        SSD1306_UpdateScreen(SSD1306_ADDR);

        if ((PIND & (1 << BTN_UP)) && (PIND & (1 << BTN_DOWN))) {
            heat_cycle(140 + 10 * maxTemp);
        }

        _delay_ms(500);
    }
}

// ---------- Helper functions ----------

//void pwm_init(void) {
//    DDRD |= (1 << MOSFET_PIN);
//    TCCR2A = (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
//    TCCR2B = (1 << CS21); // clk/8 → ~3.9 kHz
//    OCR2B = 0;
//}

void pwm_init(void) {
    DDRD |= (1 << MOSFET_PIN);   // MOSFET pin as output

    // Phase Correct PWM, OCR2B is the top for compare, COM2B1 = non-inverting
    // WGM20 = 1 (phase correct 8-bit), WGM21 = 0 -> mode = Phase Correct (TOP=0xFF)
    // No prescaler (CS20 = 1): f_pwm = F_CPU / 510 ≈ 8MHz / 510 ≈ 15.686 kHz
    TCCR2A = (1 << COM2B1) | (1 << WGM20); // non-inverting on OC2B, Phase Correct
    TCCR2B = (1 << CS20);                  // no prescaler

    OCR2B = 0;  // start off
}


void adc_init(void) {
    ADMUX = (0 << REFS1) | (1 << REFS0);   // REFS1:0 = 11 → internal 1.1V
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    DIDR0 = (1 << ADC0D) | (1 << ADC3D);
    _delay_ms(5);
}

uint16_t adc_read(uint8_t ch) {
    // Select ADC channel (preserve high bits in ADMUX)
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);

    // Allow MUX to settle
    _delay_us(20);

    // Dummy conversion (discard) to let sample capacitor charge
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    (void)ADC;

    // Real conversion
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

int get_volt(void) {
    return (adc_read(VOLT_ADC) / 14);
}

int get_temp(void) {
    return (adc_read(TEMP_ADC)/2);
}

uint8_t read_buttons(void) {
    return PIND;
}

void heat_cycle(uint8_t maxTemp) {
    SSD1306_ClearScreen();
    SSD1306_SetPosition(0, 0);
    SSD1306_DrawString("Heating...");
    SSD1306_UpdateScreen(SSD1306_ADDR);
    _delay_ms(1000);

    uint32_t timer = 0;
    const float loop_dt = 500.0f; // 500ms loop
    float pwm = 0;

    while (1) {
        char buf[20];
        int temp =  2 * get_temp();
        int volt = get_volt();

        // PID computation
        pwm = pid_update(maxTemp, temp, loop_dt);
      //  if (volt < V_MIN) pwm = 0; // safety
        OCR2B = (uint8_t)pwm;

        // Show info
        SSD1306_ClearScreen();
        SSD1306_SetPosition(0, 0);
        sprintf(buf, "T:%d  Target:%d", temp, maxTemp);
        SSD1306_DrawString(buf);

        SSD1306_SetPosition(0, 1);
        sprintf(buf, "PWM: %3d", (int)pwm);
        SSD1306_DrawString(buf);

        SSD1306_SetPosition(0, 2);
        sprintf(buf, "V: %d", volt);
        SSD1306_DrawString(buf);

        SSD1306_UpdateScreen(SSD1306_ADDR);

        // exit conditions
        //if (volt < V_MIN) break;
       // if (temp >= maxTemp) break;

        _delay_ms((int)loop_dt);
        timer += (uint32_t)loop_dt;
        if (timer > 480000) break; // 8 min timeout
        if ((PIND & (1 << BTN_UP)) || (PIND & (1 << BTN_DOWN)))break;
    }
    _delay_ms(2000);
    OCR2B = 0;
    SSD1306_ClearScreen();
    SSD1306_DrawString("Done");
    SSD1306_UpdateScreen(SSD1306_ADDR);
    _delay_ms(2000);
}


float pid_update(float setpoint, float current, float dt_ms) {
    static float integral = 0;
    static float last_error = 0;

    float error = setpoint - current;
    integral += error * (dt_ms / 1000.0f);
    float derivative = (error - last_error) / (dt_ms / 1000.0f);
    last_error = error;

    float output = (KP * error) + (KI * integral) + (KD * derivative);
    if (output > PWM_MAX) output = PWM_MAX;
    else if (output < PWM_MIN) output = PWM_MIN;

    return output;
}

