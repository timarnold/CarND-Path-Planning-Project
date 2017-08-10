#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "spline.h"
#include "json.hpp"
#include "map.h"
#include "planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

int main()
{
    uWS::Hub h;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    const double max_s = 6945.554;

    Map map(map_file_, max_s);
    Planner planner;
    // vector<double> lastTrajectory = nullptr;

    h.onMessage([&map, &planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                 uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(data);

            if (s != "")
            {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    double pos_x;
                    double pos_y;
                    double angle;
                    const int path_size = previous_path_x.size();

                    cout << "old path has " << path_size << " elemenets " << endl;

                    double s_trajectory_start = car_s;

                    for (int i = 0; i < 0 && i < previous_path_x.size(); ++i)
                    {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                        vector<double> sd = map.xyToFrenet(previous_path_x[i], previous_path_y[i], car_yaw);
                        s_trajectory_start = sd[0];
                    }

                    cout << " car speeeed is " << car_speed << endl;
                    cout << "previous trajectory has " << path_size << " points " << endl;
                    cout << "s: " << car_s << endl;
                    cout << "s: " << end_path_s << endl;
                    //   cout << "s_traj_start: " << s_trajectory_start << endl;

                    double dest_s = 50 - (s_trajectory_start - car_s);
                    double target_speed = 22.35;
                    double duration = dest_s / target_speed;

                    //   cout << "duration of time is " << duration << endl;

                    double interval = 0.02;
                    vector<double> start = {s_trajectory_start, car_speed, 0};
                    vector<double> end = {s_trajectory_start + dest_s, target_speed, 0};

                    cout << " from " << car_s << ", " << car_speed << " to " << car_s + dest_s << ", "
                         << "22.35" << endl;

                    vector<double> coeffs = planner.JMT(start, end, duration);

                    //   cout << "coeffs are: " << coeffs[0] << " " << coeffs[1] << " " << coeffs[2] << " " << coeffs[3] << "  " << coeffs[4] << " " << coeffs[5] << endl << endl;

                    double t = 0;
                    while (t < duration)
                    {
                        double s_p = coeffs[0] + coeffs[1] * t + coeffs[2] * powf(t, 2) + coeffs[3] * powf(t, 3) + coeffs[4] * powf(t, 4) + coeffs[5] * powf(t, 5);
                        vector<double> xy = map.frenetToXY(s_p, car_d);
                        next_x_vals.push_back(xy[0]);
                        next_y_vals.push_back(xy[1]);
                        t += interval;
                    }

                    cout << "current trajectory has point count: " << next_x_vals.size() << endl;

                    // lastTrajectory = coeffs;

                    //   const double dist_inc = 0.2;
                    //   for(int i = path_size; i < 50 - path_size; ++i) {
                    //       double s_p = car_s + dist_inc * i;
                    //       vector<double> xy = map.frenetToXY(s_p, car_d);
                    //       next_x_vals.push_back(xy[0]);
                    //       next_y_vals.push_back(xy[1]);
                    //   }
                    cout << endl
                         << endl;

                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
