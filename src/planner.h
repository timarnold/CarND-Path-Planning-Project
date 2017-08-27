#ifndef _PLANNER_H
#define _PLANNER_H

#include <vector>
#include "map.h"

using namespace std;

struct Planner
{
    vector<vector<double>> nextXYVals(
        Map map, int lane, double car_x, double car_y, double car_yaw, double car_s, double ref_vel,
        vector<vector<double>> previous_path) const;
};

#endif