//
// Created by wenchun on 3/17/21.
//

#ifndef XIAOTIANHYBRID_PARAM_H
#define XIAOTIANHYBRID_PARAM_H

#include "ParamHandler.hpp"
#include "Configuration.h"

class UserParameterHandler {
public:
    explicit UserParameterHandler(std::string yaml_file) : paramHandler(yaml_file) {
        bool value = true;
        value &= paramHandler.getString("urdf", urdf);
        urdf = std::string(THIS_COM) + urdf;
        value &= paramHandler.getValue("horizons", horizons);
        value &= paramHandler.getValue("ctrl_num", ctrl_num);
        value &= paramHandler.getValue("dt", dt);
        value &= paramHandler.getString("fl_foot", FL_WHEEL);
        value &= paramHandler.getString("fr_foot", FR_WHEEL);
        value &= paramHandler.getString("hl_foot", HL_WHEEL);
        value &= paramHandler.getString("hr_foot", HR_WHEEL);
        value &= paramHandler.getString("fl_hip", FL_HIP);
        value &= paramHandler.getString("fr_hip", FR_HIP);
        value &= paramHandler.getString("hl_hip", HL_HIP);
        value &= paramHandler.getString("hr_hip", HR_HIP);
        value &= paramHandler.getString("base", BASE);
        value &= paramHandler.getValue("dtMPC", dtMPC);
        value &= paramHandler.getValue("mu", mu);
        value &= paramHandler.getVector("Q_rpy", Q_rpy);
        value &= paramHandler.getVector("Q_pos", Q_pos);
        value &= paramHandler.getVector("Q_omega", Q_omega);
        value &= paramHandler.getVector("Q_vel", Q_vel);
        value &= paramHandler.getValue("rw", rw);
        value &= paramHandler.getValue("swing_height", swing_height);
        value &= paramHandler.getValue("body_height", body_height);
        value &= paramHandler.getBoolean("pub_estimatedState", pub_estimatedState);
        value &= paramHandler.getValue("P_parameter_for_test", P_parameter_for_test);
        value &= paramHandler.getValue("D_parameter_for_test", D_parameter_for_test);

        if (!value) {
            throw std::runtime_error("init param failed ...");
        }
    }

    std::string urdf, FL_WHEEL, FR_WHEEL, HL_WHEEL, HR_WHEEL,
            FL_HIP, FR_HIP, HL_HIP, HR_HIP, BASE;
    int horizons;
    int ctrl_num;
    double dt;
    double dtMPC;
    double mu;
    std::vector<double> Q_rpy, Q_pos, Q_omega, Q_vel;
    double rw;
    double swing_height, body_height;
    double P_parameter_for_test;
    double D_parameter_for_test;
    bool pub_estimatedState;
private:
    ParamHandler paramHandler;
};

#endif //XIAOTIANHYBRID_PARAM_H
