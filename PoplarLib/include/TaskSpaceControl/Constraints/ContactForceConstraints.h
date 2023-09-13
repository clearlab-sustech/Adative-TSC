//
// Created by nimapng on 6/11/21.
//

#ifndef TASKSPACECONTROL_CONTACTFORCECONSTRAINTS_H
#define TASKSPACECONTROL_CONTACTFORCECONSTRAINTS_H

#include "TaskSpaceControl/Constraints/LinearConstraints.h"

namespace TSC {
    class ContactForceConstraints : public LinearConstraints {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit ContactForceConstraints(RobotWrapper &robot, string name);

        virtual void update();

        virtual ConstMatRef C();

        virtual ConstVecRef c_lb();

        virtual ConstVecRef c_ub();

        Vec3Ref normal();

        Scalar &mu();

        Scalar &min();

        Scalar &max();

    private:
        Mat _C;
        Vec _c_lb, _c_ub;
        Vec3 _n;
        Scalar _mu, _min, _max;
    };
}


#endif //TASKSPACECONTROL_CONTACTFORCECONSTRAINTS_H
