#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "utils.h"

using namespace std;

class Navigation {

protected:
    // start in lane 1 (lane convention, inner to outer: 0, 1, 2)
    int lane_;

    // reference velocity to target
    double ref_vel_; // mph, just less than the speed limit of 50 mph

    // initialize change lane desire to false
    bool change_lanes_;

public:
    Navigation():lane_(1), ref_vel_(0), change_lanes_(false){};
    ~Navigation(){};

    void UpdateNavigation(Car& car, nlohmann::json sensor_fusion, nlohmann::json previous_path_x, double end_path_s)
    {
        // previous path size default 50
        int prev_size = previous_path_x.size();

        if(prev_size > 0)
        {
            car.UpdateS(end_path_s);
        }
        bool too_close = false;
        bool change_left_safe = true;
        bool change_right_safe = true;

        // find ref_v to use
        for(int i = 0; i< sensor_fusion.size(); i++)
        {
            // sensed car info
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            double check_car_s_next = check_car_s + ((double)prev_size*0.02*check_speed); //using prev points can project s values out in time

            // car is in my lane
            if(d < (2+4*lane_+2) && d > (2+4*lane_ -2))
            {

                // check s values greater than our car and s gap of 30m
                if((check_car_s_next > car.GetS()) && (check_car_s_next-car.GetS() < 30))
                {
                    // flag to handle collisions and cold starts
                    // also flag to try to change lanes
                    //ref_vel = 29.5; //mph
                    too_close = true;
                    change_lanes_ = true;
                }
            }

            // car is in my left lane
            if((lane_-1 >= 0) && d < (2+4*(lane_-1)+2) && d > (2+4*(lane_-1) -2))
            {
                // check s values greater than our car and s gap of 30m
                if(!(((check_car_s_next > car.GetS()) && (check_car_s_next-car.GetS() > 30)) ||
                     ((check_car_s_next < car.GetS()) && (car.GetS() - check_car_s_next > 30))))
                {
                    change_left_safe = false;
                }
            }
            // car is in my right lane
            else if((lane_ + 1 <= 2) && d < (2+4*(lane_+1)+2) && d > (2+4*(lane_+1) -2))
            {
                // check s values greater than our car and s gap of 30m
                if(!(((check_car_s_next > car.GetS()) && (check_car_s_next-car.GetS() > 30)) ||
                     ((check_car_s_next < car.GetS()) && (car.GetS() - check_car_s_next > 30))))
                {
                    change_right_safe = false;
                }
            }
        }

        // try to change lanes
        if(change_lanes_)
        {
            if(change_left[lane_] == 1 && change_left_safe)
            {
                lane_ -= 1;
                change_lanes_ = false;
            }else if(change_right[lane_] == 1 && change_right_safe)
            {
                lane_ += 1;
                change_lanes_ = false;
            }
        }

        // using flag for logic to handle collisions and cold starts
        if(too_close)
        {
            ref_vel_ -= 0.224;
        }
        else if(ref_vel_ < 49.5)
        {
            ref_vel_ += 0.224;
        }
    };

    double GetRefVel(){return ref_vel_;};
    int GetLane(){return lane_;};
    bool GetChangeLanes(){return change_lanes_;};

};

#endif // NAVIGATION_H
