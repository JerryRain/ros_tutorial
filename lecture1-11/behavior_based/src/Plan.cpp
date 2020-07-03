#include "Plan.h"
#include <iostream>

using namespace std;

Plan::Plan() : startBehavior(NULL) {

}

Behavior *Plan::getStartBehavior() {
    return startBehavior;
}

Plan::~Plan() {
 
}
