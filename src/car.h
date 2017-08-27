#ifndef _CAR_H
#define _CAR_H

#include <vector>

using namespace std;

struct Car
{
    double x;
    double y;
    double s;
    double d;
    double vx;
    double vy;

    Car(vector<double> sensor_fusion_car)
    {
        this->x = sensor_fusion_car[1];
        this->y = sensor_fusion_car[2];
        this->s = sensor_fusion_car[5];
        this->d = sensor_fusion_car[6];
        this->vx = sensor_fusion_car[3];
        this->vy = sensor_fusion_car[4];
    }

    double speed() const;
    bool in_lane(int lane) const;
};

#endif