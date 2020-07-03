#include "turn_right.h"
#include "geometry_msgs/Twist.h"

turn_right::turn_right() {
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10); // cmd_vel /cmd_vel_mux/input/teleop
    laserSub = node.subscribe("base_scan", 1, &turn_right::scanCallback, this);
    b_keep_turn_right = true;
}

bool turn_right::startCond() {
    return b_keep_turn_right;
}

void turn_right::action() {
    geometry_msgs::Twist msg;
    msg.angular.z = TURN_SPEED_MPS;
    commandPub.publish(msg);
    ROS_INFO("Turning right");
}

bool turn_right::stopCond() {
    return !b_keep_turn_right;
}

void turn_right::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // Find the closest range between the defined minimum and maximum angles
    int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
    if(minIndex<0) minIndex = 0;
    //if(maxIndex>)

    float closestRange = scan->ranges[minIndex];
    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
        //std::cout << "Range: " << closestRange << std::endl;
        if (scan->ranges[currIndex] < closestRange) {
            closestRange = scan->ranges[currIndex];
        }
    }

    // std::cout << "minIndex: " << minIndex << ", maxIndex: " << maxIndex << std::endl;
    // std::cout << "Turn right, closestRange: " << closestRange << std::endl;
    if ((closestRange < MIN_PROXIMITY_RANGE_M)|| (std::isnan(closestRange))) {
        b_keep_turn_right = true;
    } else {
        b_keep_turn_right = false;
    }
}

turn_right::~turn_right() {
}
