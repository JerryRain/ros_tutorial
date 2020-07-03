#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <vector>
//using namespace std;

class Behavior {
private:
    std::vector<Behavior *> _nextBehaviors;
    Behavior *beh;

public:
    Behavior();
    virtual bool startCond() = 0;
    virtual bool stopCond() = 0;
    virtual void action() = 0;

    Behavior *addNext(Behavior *beh);
    Behavior *selectNext();
    
    virtual ~Behavior();
};

#endif
