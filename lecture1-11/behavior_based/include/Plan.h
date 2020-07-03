#ifndef PLAN_H
#define PLAN_H

#include <vector>
#include "Behavior.h"
//using namespace std;

class Plan {
public:
    Plan();
    Behavior *getStartBehavior();
    virtual ~Plan();
protected:
    std::vector<Behavior *> behaviors;
    Behavior *startBehavior;
};

#endif
