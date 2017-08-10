#include "map.h"

constexpr double pi() { return M_PI; }

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int Waypoints::ClosestWaypointIndex(double x, double y) const
{
    double closestLen = 100000; // large number
    int closestWaypoint = 0;
    vector<double> maps_x = this->x;
    vector<double> maps_y = this->y;

    for (int i = 0; i < maps_x.size(); ++i)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }
    return closestWaypoint;
}

int Waypoints::NextWaypointIndex(double x, double y, double theta) const
{
    int closestWaypoint = ClosestWaypointIndex(x, y);
    vector<double> maps_x = this->x;
    vector<double> maps_y = this->y;

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);

    if (angle > pi() / 4)
    {
        ++closestWaypoint;
    }

    return closestWaypoint;
}

Map::Map(string map_file_, const double max_s)
{
    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line))
    {
        istringstream iss(line);
        double x, y;
        float s, d_x, d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        _waypoints.x.push_back(x);
        _waypoints.y.push_back(y);
        _waypoints.s.push_back(s);
        _waypoints.dx.push_back(d_x);
        _waypoints.dy.push_back(d_y);
    }
}

Waypoints Map::getWaypoints() const
{
    return _waypoints;
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::frenetToXY(double s, double d) const
{
    int prev_wp = -1;
    vector<double> maps_s = _waypoints.s;
    vector<double> maps_x = _waypoints.x;
    vector<double> maps_y = _waypoints.y;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
    {
        ++prev_wp;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::xyToFrenet(double x, double y, double theta) const
{
    int next_wp = _waypoints.NextWaypointIndex(x, y, theta);
    vector<double> maps_x = _waypoints.x;
    vector<double> maps_y = _waypoints.y;

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance(x_x, x_y, proj_x, proj_y);

    // see if d value is positive or negative by comparing it to a center point
    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance(center_x, center_y, x_x, x_y);
    double centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i)
    {
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}
