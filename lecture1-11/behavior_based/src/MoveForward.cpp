#include "MoveForward.h"
#include "geometry_msgs/Twist.h"

MoveForward::MoveForward() {
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);// /cmd_vel_mux/input/teleop cmd_vel
    laserSub = node.subscribe("base_scan", 1, &MoveForward::scanCallback, this);
    keepMovingForward = true;
}

bool MoveForward::startCond() {
    return keepMovingForward;
}

void MoveForward::action() {
    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED_MPS;
    commandPub.publish(msg);
    ROS_INFO("Moving forward");
}

bool MoveForward::stopCond() {
    return !keepMovingForward;
}

void MoveForward::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    if(minIndex<0) minIndex = 0;
 
    float closestRange = scan->ranges[minIndex];
    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
        //std::cout << "Range: " << closestRange << std::endl;
        if (scan->ranges[currIndex] < closestRange) {
            closestRange = scan->ranges[currIndex];
        }
    }
 
    //std::cout << "minIndex: " << minIndex << ", maxIndex: " << maxIndex << std::endl;
    //std::cout << "Move forward, closestRange: " << closestRange << std::endl;
    if ( (closestRange < MIN_PROXIMITY_RANGE_M) || (std::isnan(closestRange)) ) {
        keepMovingForward = false;
    } else {
        keepMovingForward = true;
    }
}

MoveForward::~MoveForward() {
}
