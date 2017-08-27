#ifndef _PLANNER_H
#define _PLANNER_H

#include <vector>
#include "map.h"

using namespace std;

struct Planner
{
    bool laneOpen(int lane, double car_s, int prev_size, vector<vector<double>> sensor_fusion) const;
    bool carCloseInLane(int lane, double car_s, int prev_size, vector<vector<double>> sensor_fusion) const;

    vector<vector<double>> nextXYVals(
        Map map, int lane, double car_x, double car_y, double car_yaw, double car_s, double ref_vel,
        vector<vector<double>> previous_path) const;
};

#endif