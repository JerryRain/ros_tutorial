#include "ObstacleAvoidPlan.h"
#include "MoveForward.h"
#include "TurnLeft.h"
#include "turn_right.h"
 
ObstacleAvoidPlan::ObstacleAvoidPlan(){

    // Creating behaviors
    behaviors.push_back(new MoveForward());
    behaviors.push_back(new TurnLeft());
    behaviors.push_back(new turn_right());

    std::cout << "behaviors.size(): " << behaviors.size() << std::endl;

    // Connecting behaviors
    //if(behaviors.size())
    behaviors[0]->addNext(behaviors[1]);
    behaviors[0]->addNext(behaviors[2]);

    behaviors[1]->addNext(behaviors[0]);
    behaviors[2]->addNext(behaviors[0]);

    startBehavior = behaviors[0];

}

//ObstacleAvoidPlan::~ObstacleAvoidPlan() {}


