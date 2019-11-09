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
    
    digitalWrite(_pinRed, LOW);
    digitalWrite(_pinBlue, LOW);
    digitalWrite(_pinGreen, LOW);
}

void LED::red()
{
    digitalWrite(_pinRed, HIGH);
    digitalWrite(_pinBlue, LOW);
    digitalWrite(_pinGreen, LOW);
    _red = 255;
    _blue = 0;
    _green = 0;
}

void LED::blue()
{
    digitalWrite(_pinRed, LOW);
    digitalWrite(_pinBlue, HIGH);
    digitalWrite(_pinGreen, LOW);
    _red = 0;
    _blue = 255;
    _green = 0;
}

void LED::green()
{
    digitalWrite(_pinRed, LOW);
    digitalWrite(_pinBlue, LOW);
    digitalWrite(_pinGreen, HIGH);
    _red = 0;
    _blue = 0;
    _green = 255;
}

void LED::purple()
{
    digitalWrite(_pinRed, HIGH);
    digitalWrite(_pinBlue, HIGH);
    digitalWrite(_pinGreen, LOW);
    _red = 255;
    _blue = 255;
    _green = 0;
}

void LED::white()
{
    digitalWrite(_pinRed, HIGH);
    digitalWrite(_pinBlue, HIGH);
    digitalWrite(_pinGreen, HIGH);
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
    digitalWrite(_pinRed, LOW);
    digitalWrite(_pinBlue, LOW);
    digitalWrite(_pinGreen, LOW);
}