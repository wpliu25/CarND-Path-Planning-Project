#ifndef PATH_H
#define PATH_H

#include "car.h"
#include "navigation.h"
#include "utils.h"

class Path {

protected:

public:
    Path(){};
    ~Path(){};

    void CalculatePath(Car car, Navigation navigation, vector<double>& next_x_vals, vector<double>& next_y_vals,
                       vector<double>& map_waypoints_s, vector<double>& map_waypoints_x, vector<double>& map_waypoints_y,
                       nlohmann::json previous_path_x, nlohmann::json previous_path_y)
    {

        // from navigation
        int lane = navigation.GetLane();
        double ref_vel = navigation.GetRefVel();

        // previous path size default 50
        int prev_size = previous_path_x.size();

        // list of a widely spaced (x,y) waypoints, evenly spaced at 30m to be interpolated with a spline
        vector<double> ptsx;
        vector<double> ptsy;

        // reference x, y, yaw states
        double ref_x = car.GetX();
        double ref_y = car.GetY();
        double ref_yaw = deg2rad(car.GetYaw());

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

        // in Frenet add evenly 30m spaced points ahead of the starting reference
        vector<double> next_wp0 = getXY(car.GetS()+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp1 = getXY(car.GetS()+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
        vector<double> next_wp2 = getXY(car.GetS()+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

        ptsx.push_back(next_wp0[0]);
        ptsx.push_back(next_wp1[0]);
        ptsx.push_back(next_wp2[0]);

        ptsy.push_back(next_wp0[1]);
        ptsy.push_back(next_wp1[1]);
        ptsy.push_back(next_wp2[1]);

        for(int i = 0; i < ptsx.size(); i++)
        {
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
        }

        CalculateSpline(next_x_vals, next_y_vals, ptsx, ptsy, previous_path_x, previous_path_y, ref_x, ref_y, ref_yaw, ref_vel);
    };

    void CalculateSpline(vector<double>& next_x_vals, vector<double>& next_y_vals, vector<double> ptsx, vector<double> ptsy,
                         nlohmann::json previous_path_x, nlohmann::json previous_path_y,
                         double ref_x, double ref_y, double ref_yaw, double ref_vel)
    {
        // spline
        tk::spline s;

        // set (x,y) points to the spline
        s.set_points(ptsx, ptsy);

        // start with all of the previous path points from last time
        for(int i=0; i < previous_path_x.size(); i++)
        {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
        }

        // calculate how to break up spline points so that we travel at our desired reference velocity
        double target_x = 30.0;
        double target_y = s(target_x);
        double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

        double x_add_on = 0;

        // fill up the rest of our path planner after filling it with previous points, set to 50
        for(int i=1; i <= 50-previous_path_x.size(); i++)
        {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
        }
    }

};

#endif // PATH_H
