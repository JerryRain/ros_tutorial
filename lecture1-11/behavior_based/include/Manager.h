#ifndef MANAGER_H
#define MANAGER_H

#include "Plan.h"
#include "Behavior.h"

class Manager {
public:
    Manager(Plan *plan);
    void run();
    virtual ~Manager();
private:
    Plan *plan;
    Behavior *currBehavior;
};

#endif
