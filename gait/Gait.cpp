//
// Created by wenchun on 3/22/21.
//

#include "Gait.h"

// Offset - Duration Gait
Gait::Gait(int nSegment, Vec4Int offsets, Vec4Int durations, const std::string &name) :
        _offsets(offsets.array()),
        _durations(durations.array()),
        _nIterations(nSegment),
        _iteration(0),
        _phase(0) {

    _name = name;
    // allocate memory for MPC gait table
    _mpc_table = new int[nSegment * 4];

    _offsetsDouble = offsets.cast<double>() / (double) nSegment;
    _durationsDouble = durations.cast<double>() / (double) nSegment;

    _stance = durations[0];
    _swing = nSegment - durations[0];

    swingPhase.setZero();
    stancePhase.setZero();
}


Gait::~Gait() {
    delete[] _mpc_table;
}

Vec4 Gait::getContactState() {
    return stancePhase.matrix();
}

Vec4 Gait::getSwingState() {
    return swingPhase.matrix();
}


int *Gait::getMpcTable() {
    //printf("MPC table:\n");
    for (int i = 0; i < _nIterations; i++) {
        int iter = (i + _iteration + 1) % _nIterations;
        Array4i progress = iter - _offsets;
        for (int j = 0; j < 4; j++) {
            if (progress[j] < 0) progress[j] += _nIterations;
            if (progress[j] < _durations[j])
                _mpc_table[i * 4 + j] = 1;
            else
                _mpc_table[i * 4 + j] = 0;

            //printf("%d ", _mpc_table[i*4 + j]);
        }
        //printf("\n");
    }
    return _mpc_table;
}


void Gait::run(int iterationsPerMPC, int currentIteration) {
    _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
    _phase = (double) (currentIteration % (iterationsPerMPC * _nIterations)) /
             (double) (iterationsPerMPC * _nIterations);

    Array4d leg_process = _phase - _offsetsDouble;
    for (int i(0); i < 4; i++) {
        if (leg_process[i] < 0.)
            leg_process[i] += 1;

        if (leg_process[i] <= _durationsDouble[i]) {
            stancePhase[i] = leg_process[i] / _durationsDouble[i];
            swingPhase[i] = 0.;
        } else {
            swingPhase[i] = (leg_process[i] - _durationsDouble[i]) / (1 - _durationsDouble[i]);
            stancePhase[i] = 0.;
        }
    }
}

int Gait::getCurrentGaitPhase() const {
    return _iteration;
}

double Gait::getCurrentPhaseDouble() const {
    return _phase;
}

Vec4 Gait::getSwingTime(double dtMPC) const {
    return dtMPC * (_nIterations - _durations).cast<double>();
}


Vec4 Gait::getStanceTime(double dtMPC) const {
    return dtMPC * _durations.cast<double>();
}


void Gait::debugPrint() {

}

