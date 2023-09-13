//
// Created by wenchun on 3/17/21.
//

#include "GaitScheduler.h"

GaitScheduler::GaitScheduler(const EstimatedState *estimatedState,
                             const UserCmd *userCmd,
                             GaitData *gaitData,
                             const UserParameterHandler *param) : estimatedState(estimatedState),
                                                                  userCmd(userCmd),
                                                                  gaitData(gaitData),
                                                                  param(param),
                                                                  trotting(param->horizons, Vec4Int(0, param->horizons / 2, param->horizons / 2, 0),
                                                                           Vec4Int(param->horizons / 2 + 1, param->horizons / 2 + 1, param->horizons / 2 + 1, param->horizons / 2 + 1),
                                                                           "Trotting"),
                                                                  bounding(param->horizons, Vec4Int(param->horizons / 2, param->horizons / 2, 0, 0), Vec4Int(param->horizons / 2 - 1, param->horizons / 2 - 1, param->horizons / 2 - 1, param->horizons / 2 - 1), "Bounding"),
                                                                  pronking(param->horizons, Vec4Int(0, 0, 0, 0), Vec4Int(4, 4, 4, 4), "Pronking"),
                                                                  jumping(param->horizons, Vec4Int(0, 0, 0, 0), Vec4Int(2, 2, 2, 2), "Jumping"),
                                                                  galloping(param->horizons, Vec4Int(0, 2, 7, 9), Vec4Int(4, 4, 4, 4), "Galloping"),
                                                                  standing(param->horizons, Vec4Int(0, 0, 0, 0), Vec4Int(param->horizons, param->horizons, param->horizons, param->horizons), "Standing"),
                                                                  trotRunning(param->horizons, Vec4Int(0, 5, 5, 0), Vec4Int(4, 4, 4, 4), "Trot Running"),
                                                                  walking(param->horizons, Vec4Int(0, 3, 5, 8), Vec4Int(5, 5, 5, 5), "Walking"),
                                                                  walking2(param->horizons, Vec4Int(0, 5, 5, 0), Vec4Int(7, 7, 7, 7), "Walking2"),
                                                                  pacing(param->horizons, Vec4Int(5, 0, 5, 0), Vec4Int(5, 5, 5, 5), "Pacing"),
                                                                  iter(0)
{
    currentGait = GaitTypes::NONE;

    // csvLog.open(THIS_COM"matlab/gait_data.txt");
}

void GaitScheduler::step()
{

    if (currentGait != GaitTypes(userCmd->gaitNum))
    {
        if (checkTransition())
        {
            switch (fsm.trans(userCmd->gaitNum))
            {
            case GaitTypes::STAND:
                gait = &standing;
                currentGait = GaitTypes::STAND;
                break;

            case GaitTypes::PACE:
                gait = &pacing;
                currentGait = GaitTypes::PACE;
                break;

            case GaitTypes::TROT_WALK:
                gait = &trotting;
                currentGait = GaitTypes::TROT_WALK;
                break;

            case GaitTypes::TROT_RUN:
                gait = &trotRunning;
                currentGait = GaitTypes::TROT_RUN;
                break;

            case GaitTypes::BOUND:
                gait = &bounding;
                currentGait = GaitTypes::BOUND;
                break;

            case GaitTypes::ROTARY_GALLOP:
                gait = &galloping;
                currentGait = GaitTypes::ROTARY_GALLOP;
                break;

            case GaitTypes::PRONK:
                gait = &pronking;
                currentGait = GaitTypes::PRONK;
                break;

            case GaitTypes::NONE:
                gait = nullptr;
                currentGait = GaitTypes::NONE;
                break;

            default:
                break;
            }
            iter = 0;
        }
    }

    if (gait != nullptr)
    {

        gait->run(param->dtMPC / param->dt, iter);

        Vec4 swPhase = gait->getSwingState();
        Vec4 stPhase = gait->getContactState();
        gaitData->swingTime = gait->getSwingTime(param->dtMPC);
        gaitData->nextStanceTime = gait->getStanceTime(param->dtMPC);

        for (int i(0); i < 4; i++)
        {
            if (swPhase[i] == 0. && stPhase[i] >= 0.)
            { // leg in stance
                gaitData->stanceTimeRemain[i] = gaitData->nextStanceTime[i] * (1 - stPhase[i]);
                if (gaitData->stanceTimeRemain[i] == 0.) // stance finished, swing begin
                    gaitData->swingTimeRemain[i] = gaitData->swingTime[i];
                else
                    gaitData->swingTimeRemain[i] = 0.;
            }
            else
            { // leg in swing
                gaitData->swingTimeRemain[i] = gaitData->swingTime[i] * (1 - swPhase[i]);
                gaitData->stanceTimeRemain[i] = 0.;
            }
        }

        // if (gait == &trotting)
        if (false)
        {
            int Tc = int(param->dtMPC / param->dt * param->horizons);
            int Tc_half = Tc / 2;
            if (iter % Tc == Tc_half && (!estimatedState->footState[1].contact_detected || !estimatedState->footState[2].contact_detected))
            {
                iter--;
            }
            else if (iter % Tc == 0 && (!estimatedState->footState[0].contact_detected || !estimatedState->footState[3].contact_detected))
            {
                iter--;
            }
        }

        // if (gait == &trotting) // do not use it
        if (false)
        {
            bool replan = false;
            if (gaitData->stanceTimeRemain[0] / gaitData->nextStanceTime[0] > 0.95 && !estimatedState->footState[0].contact_detected)
            {
                replan = true;
                iter = param->dtMPC / param->dt * (param->horizons - 1);
                gaitData->late_contact[0] = true;
            }
            else
            {
                if (estimatedState->footState[0].contact_detected)
                {
                    gaitData->late_contact[0] = false;
                }
            }
            if (gaitData->stanceTimeRemain[3] / gaitData->nextStanceTime[3] > 0.95 && !estimatedState->footState[3].contact_detected)
            {
                replan = true;
                iter = param->dtMPC / param->dt * (param->horizons - 1);
                gaitData->late_contact[3] = true;
            }
            else
            {
                if (estimatedState->footState[3].contact_detected)
                {
                    gaitData->late_contact[3] = false;
                }
            }
            if (gaitData->stanceTimeRemain[1] / gaitData->nextStanceTime[1] > 0.95 && !estimatedState->footState[1].contact_detected)
            {
                replan = true;
                iter = param->dtMPC / param->dt * (param->horizons / 2 - 1);
                gaitData->late_contact[1] = true;
            }
            else
            {
                if (estimatedState->footState[1].contact_detected)
                {
                    gaitData->late_contact[1] = false;
                }
            }
            if (gaitData->stanceTimeRemain[2] / gaitData->nextStanceTime[2] > 0.95 && !estimatedState->footState[2].contact_detected)
            {
                replan = true;
                iter = param->dtMPC / param->dt * (param->horizons / 2 - 1);
                gaitData->late_contact[2] = true;
            }
            else
            {
                if (estimatedState->footState[2].contact_detected)
                {
                    gaitData->late_contact[2] = false;
                }
            }
            if (replan)
            {
                printf("%ld: gait reschedule\n", iter);
                gait->run(param->dtMPC / param->dt, iter);

                Vec4 swPhase = gait->getSwingState();
                Vec4 stPhase = gait->getContactState();
                gaitData->swingTime = gait->getSwingTime(param->dtMPC);
                gaitData->nextStanceTime = gait->getStanceTime(param->dtMPC);

                for (int i(0); i < 4; i++)
                {
                    if (swPhase[i] == 0. && stPhase[i] >= 0.)
                    { // leg in stance
                        gaitData->stanceTimeRemain[i] = gaitData->nextStanceTime[i] * (1 - stPhase[i]);
                        if (gaitData->stanceTimeRemain[i] == 0.) // stance finished, swing begin
                            gaitData->swingTimeRemain[i] = gaitData->swingTime[i];
                        else
                            gaitData->swingTimeRemain[i] = 0.;
                    }
                    else
                    { // leg in swing
                        gaitData->swingTimeRemain[i] = gaitData->swingTime[i] * (1 - swPhase[i]);
                        gaitData->stanceTimeRemain[i] = 0.;
                    }
                }
            }
        }

        // printf("------------------------iter: %ld--------------------\n", iter);
        // std::cout << "swtr: " << gaitData->swingTimeRemain.transpose() << std::endl;
        // std::cout << "sttr: " << gaitData->stanceTimeRemain.transpose() << std::endl;

        // csvoutN(csvLog, gaitData->swingTimeRemain, 4, false);
        // csvoutN(csvLog, gaitData->stanceTimeRemain, 4, false);
        // csvLog << "\n";
    }

    // TODO: no gait case
    iter++;
}

bool GaitScheduler::checkTransition()
{
    if (currentGait == GaitTypes::NONE)
        return true;
    else
    {
        double minPhase, maxPhase;
        minPhase = 1 / (param->horizons * param->dtMPC / param->dt);
        maxPhase = 1 - minPhase;
        double curPhase = gait->getCurrentPhaseDouble();

        if (curPhase <= minPhase || curPhase >= maxPhase)
            return true;
        else
            return false;
    }
}
