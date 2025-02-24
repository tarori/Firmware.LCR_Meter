#pragma once

#include <stdint.h>
#include <math.h>
#include <algorithm>

struct Complex {
    double real;
    double im;
    double abs;

    Complex() : real(0.0), im(0.0), abs(0.0) {}
    Complex(double real) : real(real), im(0.0) { this->abs = fabs(real); }
    Complex(double real, double im) : real(real), im(im), abs(sqrt(real * real + im * im)) {}
    Complex(double real, double im, double abs) : real(real), im(im), abs(abs) {}

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
    double real_list[len], im_list[len];
    for (int i = 0; i < len; ++i) {
        real_list[i] = data[i].real;
        im_list[i] = data[i].im;
    }
    std::sort(&real_list[0], &real_list[len - 1]);
    std::sort(&im_list[0], &im_list[len - 1]);
    double real_med = (len % 2 == 0)
                         ? (real_list[len / 2] + real_list[len / 2 - 1]) / 2
                         : real_list[len / 2];
    double im_med = (len % 2 == 0)
                       ? (im_list[len / 2] + im_list[len / 2 - 1]) / 2
                       : im_list[len / 2];
    return Complex(real_med, im_med);
}
