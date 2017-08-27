#include "planner.h"
#include "spline.h"
#include "car.h"
#include <math.h>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

bool Planner::laneOpen(int lane, double car_s, int prev_size, vector<vector<double>> sensor_fusion) const
{
    for (int i = 0; i < sensor_fusion.size(); ++i)
    {
        vector<double> sensor_fusion_car = sensor_fusion[i];
        Car car(sensor_fusion_car);

        if (car.in_lane(lane))
        {
            double check_car_s = car.s;
            check_car_s += (double)prev_size * 0.02 * car.speed();
            if (fabs(check_car_s - car_s) < 30)
            {
                return false;
            }
        }
    }
    return true;
}

bool Planner::carCloseInLane(int lane, double car_s, int prev_size, vector<vector<double>> sensor_fusion) const
{
    for (int i = 0; i < sensor_fusion.size(); ++i)
    {
        vector<double> sensor_fusion_car = sensor_fusion[i];
        Car car(sensor_fusion_car);

        if (car.in_lane(lane))
        {
            double check_speed = car.speed();
            double check_car_s = car.s;

            check_car_s += (double)prev_size * 0.02 * car.speed();
            if (check_car_s > car_s && (check_car_s - car_s) < 30)
            {
                return true;
            }
        }
    }
    return false;
}

vector<vector<double>> Planner::nextXYVals(
    Map map,
    int lane,
    double car_x,
    double car_y,
    double car_yaw,
    double car_s,
    double ref_vel,
    vector<vector<double>> previous_path) const
{
    vector<double> previous_path_x = previous_path[0];
    vector<double> previous_path_y = previous_path[1];
    int prev_size = previous_path_x.size();

    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    if (prev_size < 2)
    {
        double prev_car_x = car_x - cos(ref_yaw);
        double prev_car_y = car_y - sin(ref_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    }
    else
    {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    vector<double> next_wp0 = map.frenetToXY(car_s + 30, 2 + 4 * lane);
    vector<double> next_wp1 = map.frenetToXY(car_s + 60, 2 + 4 * lane);
    vector<double> next_wp2 = map.frenetToXY(car_s + 90, 2 + 4 * lane);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); ++i)
    {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for (int i = 0; i < prev_size; ++i)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    tk::spline s;
    s.set_points(ptsx, ptsy);
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double x_add_on = 0;

    for (int i = 1; i <= 50 - prev_size; ++i)
    {
        double N = target_dist / (0.02 * ref_vel / 2.24);
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    vector<vector<double>> nextXY;
    nextXY.push_back(next_x_vals);
    nextXY.push_back(next_y_vals);
    return nextXY;
}
