#include <ros/ros.h>
#include "ObstacleAvoidPlan.h"
#include "Manager.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "behavior_based_wanderer");

    ObstacleAvoidPlan plan;
    //ObstacleAvoidPlan *plan = new ObstacleAvoidPlan();
    Manager manager(&plan);
    //Manager *manager = new Manager(plan);

    // Start the movement
    manager.run();
    //manager->run();

    ROS_INFO("run main stoped");
    return 0;
}
