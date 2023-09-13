//
// Created by nimapng on 1/4/22.
//

#ifndef CLIMBSTAIRS_PERCEPTIONPROCESSOR_H
#define CLIMBSTAIRS_PERCEPTIONPROCESSOR_H

#include "DataSets.h"

class PerceptionProcessor {
public:
    PerceptionProcessor(PerceptionData *perceptionDataCurrent, PerceptionData *perceptionDataProcessed);

    void process();

private:
    PerceptionData *perceptionDataCurrent;
    PerceptionData *perceptionDataProcessed;
    
};


#endif //CLIMBSTAIRS_PERCEPTIONPROCESSOR_H
