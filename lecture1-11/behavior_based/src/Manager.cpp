#include "Manager.h"
#include <ros/ros.h>

Manager::Manager(Plan *plan) : plan(plan) {
    currBehavior = plan->getStartBehavior();
}
Manager::~Manager() {
}

void Manager::run()
{
    ROS_INFO("Manager started");
    ros::Rate rate(10);
    if (!currBehavior->startCond()) {
        ROS_ERROR("Cannot start the first behavior");
        return;
    }
    while (ros::ok() && currBehavior != NULL) {
        currBehavior->action();

        if (currBehavior->stopCond()) {
            currBehavior = currBehavior->selectNext();
        }

        if(currBehavior == NULL)
            ROS_INFO("currBehavior is NULL");

        ros::spinOnce();
        //ros::spin();
        rate.sleep();
    }
    ROS_INFO("Manager stopped");
}
