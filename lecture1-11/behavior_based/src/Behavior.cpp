#include "Behavior.h"

#include <iostream>
#include <vector>

using namespace std;

Behavior::Behavior() {
}

Behavior::~Behavior() {
}

Behavior *Behavior::addNext(Behavior *beh) {
    _nextBehaviors.push_back(beh);
    return this;
}

Behavior *Behavior::selectNext() {
    for (int i = 0; i < _nextBehaviors.size(); i++)
    {
        if (_nextBehaviors[i]->startCond())
            return _nextBehaviors[i];
    }
    return NULL;
}
