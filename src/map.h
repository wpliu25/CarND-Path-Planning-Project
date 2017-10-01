#ifndef MAP_H
#define MAP_H

#include <vector>

using namespace std;

class Map {

protected:

    vector<double> map_waypoints_x_;
    vector<double> map_waypoints_y_;
    vector<double> map_waypoints_s_;
    vector<double> map_waypoints_dx_;
    vector<double> map_waypoints_dy_;

public:
    Map(){};
    Map(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy):
        map_waypoints_x_(map_waypoints_x), map_waypoints_y_(map_waypoints_y), map_waypoints_s_(map_waypoints_s), map_waypoints_dx_(map_waypoints_dx), map_waypoints_dy_(map_waypoints_dy){};
    ~Map(){};

    vector<double> GetMapWaypointsS(){return map_waypoints_s_;};
    vector<double> GetMapWaypointsX(){return map_waypoints_x_;};
    vector<double> GetMapWaypointsY(){return map_waypoints_y_;};
};

#endif // MAP_H
