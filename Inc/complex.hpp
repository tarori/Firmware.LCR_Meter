#pragma once

#include <stdint.h>
#include <math.h>

struct Complex {
    float real;
    float im;
    float abs;

    Complex() : real(0.0f), im(0.0f), abs(0.0f) {}
    Complex(float real) : real(real), im(0.0f) { this->abs = fabs(real); }
    Complex(float real, float im) : real(real), im(im), abs(sqrt(real * real + im * im)) {}
    Complex(float real, float im, float abs) : real(real), im(im), abs(abs) {}

    Complex operator+(Complex b)
    {
        return Complex(this->real + b.real, this->im + b.im);
    }

    Complex operator-(Complex b)
    {
        return Complex(this->real - b.real, this->im - b.im);
    }

    Complex operator*(Complex b)
    {
        struct Complex y;
        y.real = this->real * b.real - this->im * b.im;
        y.im = this->im * b.real + this->real * b.im;
        y.abs = this->abs * b.abs;
        return y;
    }

    Complex operator/(Complex b)
    {
        struct Complex y;
        y.real = (this->real * b.real + this->im * b.im) / (b.abs * b.abs);
        y.im = (this->im * b.real - this->real * b.im) / (b.abs * b.abs);
        y.abs = this->abs / b.abs;
        return y;
    }
};
