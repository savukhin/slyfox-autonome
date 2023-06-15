#pragma once

#include <math.h>

class Arrow
{
private:
    double x_;
    double y_;

public:
    Arrow(double x = 0, double y = 0): x_(x), y_(y) {}

    double getX() { return x_; }
    double getY() { return y_; }

    double length() const {
        return sqrt(x_ * x_ + y_ * y_); 
    }
};
