	#include <reg52.h>
#include <stdio.h>
#include <LCD_8_bit.h>

#define SOUND_VELOCITY 34300
#define CLOCK_PERIOD 1.085e-6 

// Left Sensor
sbit TRIG_LEFT  = P3^0;
sbit ECHO_LEFT  = P3^1;

// Right Sensor
sbit TRIG_RIGHT = P3^2;
sbit ECHO_RIGHT = P3^3;

// Indicators
sbit RED_LED     = P1^3;
sbit YELLOW_LED  = P1^4;
sbit GREEN_LED   = P1^5;

void delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 112; j++);
}

void delay_us() {
    TL0 = 0xF5;
    TH0 = 0xFF;
    TR0 = 1;
    while (TF0 == 0);
    TR0 = 0;
    TF0 = 0;
}

void init_timer() {
    TMOD = 0x01;
    TF0 = 0;
    TR0 = 0;
}

void send_trigger_left() {
    TRIG_LEFT = 1;
    delay_us();
    TRIG_LEFT = 0;
}

void send_trigger_right() {
    TRIG_RIGHT = 1;
    delay_us();
    TRIG_RIGHT = 0;
}

unsigned int measure_distance_left() {
    float value = CLOCK_PERIOD * SOUND_VELOCITY;
    unsigned int ticks;
    float distance;

    send_trigger_left();
    while (!ECHO_LEFT);
    TR0 = 1;
    while (ECHO_LEFT && !TF0);
    TR0 = 0;

    ticks = (TH0 << 8) | TL0;
    distance = (ticks * value) / 2.0;

    return (unsigned int)distance;
}

unsigned int measure_distance_right() {
    float value = CLOCK_PERIOD * SOUND_VELOCITY;
    unsigned int ticks;
    float distance;

    send_trigger_right();
    while (!ECHO_RIGHT);
    TR0 = 1;
    while (ECHO_RIGHT && !TF0);
    TR0 = 0;

    ticks = (TH0 << 8) | TL0;
    distance = (ticks * value) / 2.0;

    return (unsigned int)distance;
}

void display_distance(unsigned int left, unsigned int right) {
    char buf[16];
    LCD_String_xy(1, 0, "L:");
    sprintf(buf, "%3d cm", left);
    LCD_String(buf);

    LCD_String_xy(2, 0, "R:");
    sprintf(buf, "%3d cm", right);
    LCD_String(buf);
}

void control_alerts(unsigned int distance) {
    if (distance < 20) {
        RED_LED = 1;
        YELLOW_LED = 0;
        GREEN_LED = 0;
    } else if (distance < 50) {
        RED_LED = 0;
        YELLOW_LED = 1;
        GREEN_LED = 0;
    } else if (distance <= 100) {
        RED_LED = 0;
        YELLOW_LED = 0;
        GREEN_LED = 1;
    } else {
        RED_LED = 0;
        YELLOW_LED = 0;
        GREEN_LED = 0;
    }
}

void main() {
    unsigned int dist_left, dist_right, min_dist;

    LCD_Init();
    init_timer();

    while (1) {
        dist_left = measure_distance_left();
        dist_right = measure_distance_right();

        min_dist = (dist_left < dist_right) ? dist_left : dist_right;

        display_distance(dist_left, dist_right);
        control_alerts(min_dist);

        delay_ms(200);
    }
}
