#include <Arduino.h>
#include "LED.h"

LED::LED(int pinRed, int pinBlue, int pinGreen)
{
    pinMode(pinRed, OUTPUT);
    pinMode(pinBlue, OUTPUT);
    pinMode(pinGreen, OUTPUT);
    
    _pinRed = pinRed;
    _pinBlue = pinBlue;
    _pinGreen = pinGreen;
    _red = 255;
    _blue = 255;
    _green = 255;
    
    analogWrite(_pinRed, 0);
    analogWrite(_pinBlue, 0);
    analogWrite(_pinGreen, 0);
}

void LED::red()
{
    analogWrite(_pinRed, 255);
    analogWrite(_pinBlue, 0);
    analogWrite(_pinGreen, 0);
    _red = 255;
    _blue = 0;
    _green = 0;
}

void LED::blue()
{
    analogWrite(_pinRed, 0);
    analogWrite(_pinBlue, 255);
    analogWrite(_pinGreen, 0);
    _red = 0;
    _blue = 255;
    _green = 0;
}

void LED::green()
{
    analogWrite(_pinRed, 0);
    analogWrite(_pinBlue, 0);
    analogWrite(_pinGreen, 255);
    _red = 0;
    _blue = 0;
    _green = 255;
}

void LED::purple()
{
    analogWrite(_pinRed, 255);
    analogWrite(_pinBlue, 255);
    analogWrite(_pinGreen, 0);
    _red = 255;
    _blue = 255;
    _green = 0;
}

void LED::white()
{
    analogWrite(_pinRed, 255);
    analogWrite(_pinBlue, 255);
    analogWrite(_pinGreen, 255);
    _red = 255;
    _blue = 255;
    _green = 255;
}

void LED::rgb(int r, int g, int b)
{
    analogWrite(_pinRed, r);
    analogWrite(_pinBlue, b);
    analogWrite(_pinGreen, g);
    _red = r;
    _blue = b;
    _green = g;
}

void LED::on() //resume previous state
{
    analogWrite(_pinRed, _red);
    analogWrite(_pinBlue, _blue);
    analogWrite(_pinGreen, _green);
}

void LED::off()
{
    analogWrite(_pinRed, 0);
    analogWrite(_pinBlue, 0);
    analogWrite(_pinGreen, 0);
}