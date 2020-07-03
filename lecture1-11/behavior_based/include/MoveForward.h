#ifndef MOVE_FORWARD_H
#define MOVE_FORWARD_H

#include "Behavior.h"
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <cmath>

class MoveForward: public Behavior {
public:
    MoveForward();
    virtual bool startCond();
    virtual void action();
    virtual bool stopCond();
    virtual ~MoveForward();
private:
    double FORWARD_SPEED_MPS = 0.5;
    double MIN_SCAN_ANGLE_RAD = -30.0/180 * M_PI;
    double MAX_SCAN_ANGLE_RAD = +30.0/180 * M_PI;
    float MIN_PROXIMITY_RANGE_M = 1.0;

    ros::NodeHandle node;
    ros::Publisher commandPub;
    ros::Subscriber laserSub;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool keepMovingForward;
};

#endif
