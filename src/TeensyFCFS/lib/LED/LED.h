/*
    LED.h - Library for controlling an RGB Led
    Created by olive, November 8, 2019
*/

#ifndef LED_h
#define LED_h

class LED
{
    public:
        LED(int pinRed, int pinBlue, int pinGreen);
        void red();
        void blue();
        void green();
        void purple();
        void white();
        void rgb(int r, int g, int b);
        void on();
        void off();

    private:
        int _pinRed;
        int _pinBlue;
        int _pinGreen;
        int _red;
        int _blue;
        int _green;
};

#endif