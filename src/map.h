#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>
#include <math.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

struct Waypoints
{
    vector<double> x;
    vector<double> y;
    vector<double> s;
    vector<double> dx;
    vector<double> dy;

    int ClosestWaypointIndex(double x, double y) const;
    int NextWaypointIndex(double x, double y, double theta) const;
};

class Map
{
    Map(){};
    Waypoints _waypoints;

  public:
    Map(string map_file_, const double max_s);
    Waypoints getWaypoints() const;
    vector<double> xyToFrenet(double x, double y, double theta) const;
    vector<double> frenetToXY(double s, double d) const;
};

#endif
