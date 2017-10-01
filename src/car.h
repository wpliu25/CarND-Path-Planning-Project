#ifndef CAR_H
#define CAR_H

#include <vector>

using namespace std;

class Car {

protected:

    int id_;
    double x_;
    double y_;
    double s_;
    double d_;
    double yaw_;
    double vx_;
    double vy_;

public:
    Car(){};
    Car(int id, double x, double y, double s, double d, double yaw, double vx = 0, double vy = 0)
    {
        id_ = id;
        x_ = x;
        y_ = y;
        s_ = s;
        d_ = d;
        yaw_ = yaw;
        if(vx_ != vx) vx_ = vx;
        if(vy_ != vy) vx_ = vy;
    };
    ~Car(){};

    void UpdateCar(double x, double y, double s, double d, double yaw, double vx = 0, double vy = 0)
    {
        x_ = x;
        y_ = y;
        s_ = s;
        d_ = d;
        yaw_ = yaw;
        if(vx_ != vx) vx_ = vx;
        if(vy_ != vy) vx_ = vy;
    };

    void UpdateS(double s){s_ = s;};

    double GetX(){return x_;};
    double GetY(){return y_;};
    double GetS(){return s_;};
    double GetYaw(){return yaw_;};

};

#endif // CAR_H
