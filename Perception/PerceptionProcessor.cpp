//
// Created by nimapng on 1/4/22.
//

#include "PerceptionProcessor.h"

PerceptionProcessor::PerceptionProcessor(PerceptionData *perceptionDataCurrent,
                                         PerceptionData *perceptionDataProcessed) :
        perceptionDataCurrent(perceptionDataCurrent),
        perceptionDataProcessed(perceptionDataProcessed) {

}


void PerceptionProcessor::process() {
    perceptionDataCurrent->mtx.lock();
    perceptionDataCurrent->mtx.unlock();
}