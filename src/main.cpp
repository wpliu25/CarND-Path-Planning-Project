#include "utils.h"
#include "spline.h"
#include "car.h"
#include "path.h"

int main() {
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    // start in lane 1 (lane convention, inner to outer: 0, 1, 2)
    int lane = 1;

    // reference velocity to target
    double ref_vel = 0; // mph, just less than the speed limit of 50 mph

    // initialize change lane desire to false
    bool change_lanes = false;

    // indexed by lanes, boolean flags to indicate valid left or right lane change
    vector<int> change_left;
    vector<int> change_right;
    change_left.push_back(0);
    change_left.push_back(1);
    change_left.push_back(1);
    change_right.push_back(1);
    change_right.push_back(1);
    change_right.push_back(0);
    Car car;
    Path path;

    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &change_lanes, &change_left, &change_right, &car, &path](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(data);

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    //double car_speed = j[1]["speed"];
                    //----------
                    // Car
                    //----------
                    car.UpdateCar(car_x, car_y, car_s, car_d, car_yaw);

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;


                    // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                    // previous path size default 50
                    int prev_size = previous_path_x.size();

                    //===============
                    // sensor fusion
                    //===============
                    //----------------------------------------------------------------------------------------
                    if(prev_size > 0)
                    {
                        car.UpdateS(end_path_s);
                    }
                    bool too_close = false;
                    bool change_left_safe = true;
                    bool change_right_safe = true;
                    int desired_path_length = 50;

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
                        if(d < (2+4*lane+2) && d > (2+4*lane -2))
                        {

                            // check s values greater than our car and s gap of 30m
                            if((check_car_s_next > car.GetS()) && (check_car_s_next-car.GetS() < 30))
                            {
                                // flag to handle collisions and cold starts
                                // also flag to try to change lanes
                                //ref_vel = 29.5; //mph
                                too_close = true;
                                change_lanes = true;
                            }
                        }

                        // car is in my left lane
                        if((lane-1 >= 0) && d < (2+4*(lane-1)+2) && d > (2+4*(lane-1) -2))
                        {
                            // check s values greater than our car and s gap of 30m
                            if(!(((check_car_s_next > car.GetS()) && (check_car_s_next-car.GetS() > 30)) ||
                                 ((check_car_s_next < car.GetS()) && (car.GetS() - check_car_s_next > 30))))
                            {
                                change_left_safe = false;
                            }
                        }
                        // car is in my right lane
                        else if((lane + 1 <= 2) && d < (2+4*(lane+1)+2) && d > (2+4*(lane+1) -2))
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
                    if(change_lanes)
                    {
                        if(change_left[lane] == 1 && change_left_safe)
                        {
                            lane -= 1;
                            desired_path_length = 50;
                            change_lanes = false;
                        }else if(change_right[lane] == 1 && change_right_safe)
                        {
                            lane += 1;
                            desired_path_length = 50;
                            change_lanes = false;
                        }
                    }

                    // using flag for logic to handle collisions and cold starts
                    if(too_close)
                    {
                        ref_vel -= 0.224;
                    }
                    else if(ref_vel < 49.5)
                    {
                        ref_vel += 0.224;
                    }

                    //================================================================================================
                    //----------
                    // Path
                    //----------

                    // list of a widely spaced (x,y) waypoints, evenly spaced at 30m to be interpolated with a spline
                    vector<double> ptsx;
                    vector<double> ptsy;

                    // reference x, y, yaw states
                    double ref_x = car.GetX();
                    double ref_y = car.GetY();
                    double ref_yaw = deg2rad(car.GetYaw());

                    path.CalculatePath(car, ptsx, ptsy, ref_x, ref_y, ref_yaw, prev_size, previous_path_x, previous_path_y);

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

                    // spline
                    tk::spline s;

                    // set (x,y) points to the spline
                    s.set_points(ptsx, ptsy);

                    // define the actual (x,y) points we will use for the planner
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

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
                    for(int i=1; i <= desired_path_length-previous_path_x.size(); i++)
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

                    // END
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
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
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
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
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
