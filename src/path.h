#ifndef PATH_H
#define PATH_H

#include "car.h"
#include "utils.h"

using namespace std;
// for convenience
using json = nlohmann::json;

class Path {

protected:

    int id_;
    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double vx_;
    double vy_;

    vector<double> previous_s_;
    vector<double> previous_d_;

public:
    Path(){};
    ~Path(){};

    void CalculatePath(Car car, vector<double>& ptsx, vector<double>& ptsy, double &ref_x, double &ref_y, double & ref_yaw, int prev_size, nlohmann::json previous_path_x, nlohmann::json previous_path_y)
    {
        // if previous size is almost empty, use the car as starting reference
        if(prev_size < 2)
        {
            // use 2 points that make the path tanget to the car
            double prev_car_x = car.GetX() - cos(car.GetYaw());
            double prev_car_y = car.GetY() - sin(car.GetYaw());

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car.GetX());

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car.GetY());
        }
        // else use the previous path's end point as starting reference
        else
        {
            // redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // use 2 points that make the path tangent to the previous path's end point

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
        }
    };

};

#endif // PATH_H
