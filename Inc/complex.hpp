#pragma once

#include <stdint.h>
#include <math.h>
#include <algorithm>

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

Complex mid(Complex data[], const int len)
{
    float real_list[len], im_list[len];
    for (int i = 0; i < len; ++i) {
        real_list[i] = data[i].real;
        im_list[i] = data[i].im;
    }
    std::sort(&real_list[0], &real_list[len - 1]);
    std::sort(&im_list[0], &im_list[len - 1]);
    float real_med = (len % 2 == 0)
                         ? (real_list[len / 2] + real_list[len / 2 - 1]) / 2
                         : real_list[len / 2];
    float im_med = (len % 2 == 0)
                       ? (im_list[len / 2] + im_list[len / 2 - 1]) / 2
                       : im_list[len / 2];
    return Complex(real_med, im_med);
}
