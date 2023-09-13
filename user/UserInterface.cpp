//
// Created by wenchun on 3/26/21.
//

#include "UserInterface.h"

UserInterface::UserInterface(UserParameterHandler *param,
                             UserCmd *userCmd) : param(param),
                                                 userCmd(userCmd)
{
    joystick = std::make_shared<Joystick>("/dev/input/js0");
    if (!joystick->isFound())
        printf("joystick not found!\n");
}

void UserInterface::update(const EstimatedState *estimatedState)
{
    // update joystick state at 60Hz
    if (iter % 1 / (60 * param->dt) == 0)
        joystick->updateCommand(&event, gameCmd);
    double temp_body_height = param->body_height;
    // control mode
    if (param->ctrl_num == 1 && gameCmd.BACK && userCmd->gaitNum == 1)
    {
        // std::cout << "back" << std::endl;
        param->body_height -= 0.0003;
        if (param->body_height <= 0.1)
            param->body_height = 0.1;
    }
    else if (param->ctrl_num == 1 && gameCmd.START && userCmd->gaitNum == 1)
    {
        // std::cout << "start" << std::endl;
        param->body_height += 0.0003;
        if (param->body_height >= 0.38)
            param->body_height = 0.38;
    }

    // gait
    if (param->ctrl_num == 1 && gameCmd.LB && gameCmd.X && userCmd->gaitNum == 1 && estimatedState->floatingBaseState.pos.z() > 0.30)
    {
        // std::cout << "LB+X" << std::endl;
        userCmd->gaitNum = 2; // trotting
    }
    else if (param->ctrl_num == 1 && gameCmd.LB && gameCmd.A)
    {
        // std::cout << "LB+A" << std::endl;
        userCmd->gaitNum = 1; // stance
        userCmd->vx_des = 0;
        userCmd->vy_des = 0;
        userCmd->yawd_des = 0;
        fixed_vel = false;
    }
    else if (param->ctrl_num == 11 && gameCmd.LB && gameCmd.Y)
    {
        // std::cout << "LB+Y" << std::endl;
        userCmd->gaitNum = 1; // stance
        param->ctrl_num = 1;
        userCmd->vx_des = 0;
        userCmd->vy_des = 0;
        userCmd->yawd_des = 0;
        fixed_vel = false;
    }
    else if (gameCmd.RB && gameCmd.B && estimatedState->floatingBaseState.pos.z() - estimatedState->footState[0].pos.z() < 0.20)
    {
        // std::cout << "RB+Y" << std::endl;
        userCmd->gaitNum = 1; // stance
        param->ctrl_num = 11;
        userCmd->vx_des = 0;
        userCmd->vy_des = 0;
        userCmd->yawd_des = 0;
    }
    else if (gameCmd.RB && gameCmd.A && param->ctrl_num == 1 && userCmd->gaitNum == 2)
    {
        if (!fixed_vel)
        {
            userCmd->vx_des = 0.45;
            fixed_vel = true;
            userCmd->vy_des = 0;
            userCmd->yawd_des = 0;
        }
    }
    else if (gameCmd.RB && gameCmd.X && param->ctrl_num == 1 && userCmd->gaitNum == 2)
    {
        if (fixed_vel)
        {
            userCmd->vx_des = 0.0;
            fixed_vel = false;
            userCmd->vy_des = 0;
            userCmd->yawd_des = 0;
        }
    }

    if (param->ctrl_num == 11)
    {
        param->body_height = estimatedState->floatingBaseState.pos.z();
    }

    // command
    if (userCmd->gaitNum == 2 && !fixed_vel)
    {
        double filter = 0.005;
        double left_0 = abs(gameCmd.leftStickAnalog[0]) > 0.1 ? gameCmd.leftStickAnalog[0] : 0;
        double left_1 = abs(gameCmd.leftStickAnalog[1]) > 0.1 ? gameCmd.leftStickAnalog[1] : 0;
        double right_1 = abs(gameCmd.rightStickAnalog[1]) > 0.1 ? gameCmd.rightStickAnalog[1] : 0;

        userCmd->vx_des = filter * left_0 + (1 - filter) * userCmd->vx_des;
        userCmd->vy_des = filter * left_1 / 2 + (1 - filter) * userCmd->vy_des;
        userCmd->yawd_des = filter * right_1 + (1 - filter) * userCmd->yawd_des;
        // std::cout << "yawd_des = " << gameCmd.leftStickAnalog << std::endl;
    }

    // if (param->body_height - temp_body_height > 0.0000001)
    // {
    //     param->body_height = temp_body_height + 0.0000001;
    // }
    // else if (param->body_height - temp_body_height < -0.0000001)
    // {
    //     param->body_height = temp_body_height - 0.0000001;
    // }
    if (iter < 1000)
    {
        param->ctrl_num = 11;
        userCmd->gaitNum = 1;
        userCmd->vx_des = 0;
        userCmd->vy_des = 0;
        userCmd->yawd_des = 0;
    }

    if (iter > 2000)
    {
        param->ctrl_num = 1;
        param->body_height = 0.38;
    }

    if (iter > 4000)
    {
        userCmd->gaitNum = 2;
    }

    if (iter > 5000)
    {
        userCmd->vx_des = 0.45;
        // userCmd->vx_des = 0.85 * cos(0.005 * iter);
        // userCmd->vy_des = 0.8 * cos(0.005 * iter);
        if (iter % 1000 < 200)
        {
            userCmd->yawd_des = 1.4 * sin(0.01 * (iter % 1000));
        }
    }
    iter++;
}
