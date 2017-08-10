#ifndef _PLANNER_H
#define _PLANNER_H

#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/LU"
#include <vector>

using namespace std;

struct Planner {
    vector<double> JMT(vector<double> start, vector<double> end, double T) const;
};

#endif