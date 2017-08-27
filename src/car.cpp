#include <math.h>
#include "car.h"

double Car::speed() const
{
    double vx = this->vx;
    double vy = this->vy;
    return sqrt(vx * vx + vy * vy);
}

bool Car::in_lane(int lane) const
{
    double d = this->d;
    return (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2));
}