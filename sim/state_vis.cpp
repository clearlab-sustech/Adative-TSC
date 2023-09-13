// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "Configuration.h"
#include "StateVis.hpp"

#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <chrono>
#include <lcm/lcm-cpp.hpp>

#include "cppTypes.h"
#include "PerceptionData.h"
#include "PerceptionTerrain_lcm.hpp"
#include "DebugInfo_lcm.hpp"
#include <deque>
#include "Timer.h"
#include "PerceptionLocation_lcm.hpp"
#include "orientation_tools.h"

using namespace ori;
#define MAX_PLANE_NUM 30
#define SAVE_PLANE

//-------------------------------- global -----------------------------------------------
#define SET_INIT_QPOS
double q_init[] = {0, 0, 0.057, 1, 0, 0, 0,
                   0.577, 1.21, -2.78,
                   -0.577, 1.21, -2.78,
                   0.577, 1.21, -2.78,
                   -0.577, 1.21, -2.78};

std::fstream save_plane;
// lcm message
StateVis stateVis;
lcm::LCM lcm_rec;

std::array<Plane, MAX_PLANE_NUM> perception_planes;
int plane_num = 0;
bool foothold_planner_state = false;
int swing_planner_state = 0;
double foot_planner_time = 0;
double cmd_velocity = 0;
double planned_velocity[3];
Vec3 vbody_real;
DebugInfo_lcm received_debuginfo;
bool plane_received = false;

// constants
const int maxgeom = 5000;         // preallocated geom array in mjvScene
const double syncmisalign = 0.1;  // maximum time mis-alignment before re-sync
const double refreshfactor = 0.7; // fraction of refresh available for simulation

// model and data
mjModel *m = NULL;
mjData *d = NULL;
char filename[1000] = "";

// abstract visualization
mjvScene scn;
mjvCamera cam;
mjvOption vopt;
mjvPerturb pert;
mjvFigure figconstraint;
mjvFigure figcost;
mjvFigure figtimer;
mjvFigure figsize;
mjvFigure figsensor;
mjvFigure figstate;
mjvFigure figtimecost;
mjvFigure figvelocity;

// OpenGL rendering and UI
GLFWvidmode vmode;
int windowpos[2];
int windowsize[2];
mjrContext con;
GLFWwindow *window = NULL;
mjuiState uistate;
mjUI ui0, ui1;

// UI settings not contained in MuJoCo structures
struct
{
    // file
    int exitrequest = 0;

    // option
    int spacing = 0;
    int color = 2;
    int font = 0;
    int ui0 = 0;
    int ui1 = 0;
    int help = 0;
    int info = 1;
    int profiler = 0;
    int sensor = 1;
    int fullscreen = 0;
    int vsync = 1;
    int busywait = 0;

    // simulation
    int run = 1;
    int key = 0;
    int loadrequest = 0;

    // watch
    char field[mjMAXUITEXT] = "qpos";
    int index = 0;

    // physics: need sync
    int disable[mjNDISABLE];
    int enable[mjNENABLE];

    // rendering: need sync
    int camera = 3;
} settings;

// section ids
enum
{
    // left ui
    SECT_FILE = 0,
    SECT_OPTION,
    SECT_SIMULATION,
    SECT_WATCH,
    SECT_PHYSICS,
    SECT_RENDERING,
    SECT_GROUP,
    NSECT0,

    // right ui
    SECT_JOINT = 0,
    SECT_CONTROL,
    NSECT1
};

// file section of UI
const mjuiDef defFile[] =
    {
        {mjITEM_SECTION, "File", 1, NULL, "AF"},
        {mjITEM_BUTTON, "Save xml", 2, NULL, ""},
        {mjITEM_BUTTON, "Save mjb", 2, NULL, ""},
        {mjITEM_BUTTON, "Print model", 2, NULL, "CM"},
        {mjITEM_BUTTON, "Print data", 2, NULL, "CD"},
        {mjITEM_BUTTON, "Quit", 1, NULL, "CQ"},
        {mjITEM_END}};

// option section of UI
const mjuiDef defOption[] =
    {
        {mjITEM_SECTION, "Option", 1, NULL, "AO"},
        {mjITEM_SELECT, "Spacing", 1, &settings.spacing, "Tight\nWide"},
        {mjITEM_SELECT, "Color", 1, &settings.color, "Default\nOrange\nWhite\nBlack"},
        {mjITEM_SELECT, "Font", 1, &settings.font, "50 %\n100 %\n150 %\n200 %\n250 %\n300 %"},
        {mjITEM_CHECKINT, "Left UI (Tab)", 1, &settings.ui0, " #258"},
        {mjITEM_CHECKINT, "Right UI", 1, &settings.ui1, "S#258"},
        {mjITEM_CHECKINT, "Help", 2, &settings.help, " #290"},
        {mjITEM_CHECKINT, "Info", 2, &settings.info, " #291"},
        {mjITEM_CHECKINT, "Profiler", 2, &settings.profiler, " #292"},
        {mjITEM_CHECKINT, "Sensor", 2, &settings.sensor, " #293"},
#ifdef __APPLE__
        {mjITEM_CHECKINT, "Fullscreen", 0, &settings.fullscreen, " #294"},
#else
        {mjITEM_CHECKINT, "Fullscreen", 1, &settings.fullscreen, " #294"},
#endif
        {mjITEM_CHECKINT, "Vertical Sync", 1, &settings.vsync, " #295"},
        {mjITEM_CHECKINT, "Busy Wait", 1, &settings.busywait, " #296"},
        {mjITEM_END}};

// simulation section of UI
const mjuiDef defSimulation[] =
    {
        {mjITEM_SECTION, "Simulation", 1, NULL, "AS"},
        {mjITEM_RADIO, "", 2, &settings.run, "Pause\nRun"},
        {mjITEM_BUTTON, "Reset", 2, NULL, " #259"},
        {mjITEM_BUTTON, "Reload", 2, NULL, "CL"},
        {mjITEM_BUTTON, "Align", 2, NULL, "CA"},
        {mjITEM_BUTTON, "Copy pose", 2, NULL, "CC"},
        {mjITEM_SLIDERINT, "Key", 3, &settings.key, "0 0"},
        {mjITEM_BUTTON, "Reset to key", 3},
        {mjITEM_BUTTON, "Set key", 3},
        {mjITEM_END}};

// watch section of UI
const mjuiDef defWatch[] =
    {
        {mjITEM_SECTION, "Watch", 0, NULL, "AW"},
        {mjITEM_EDITTXT, "Field", 2, settings.field, "qpos"},
        {mjITEM_EDITINT, "Index", 2, &settings.index, "1"},
        {mjITEM_STATIC, "Value", 2, NULL, " "},
        {mjITEM_END}};

// help strings
const char help_content[] =
    "Alt mouse button\n"
    "UI right hold\n"
    "UI title double-click\n"
    "Space\n"
    "Esc\n"
    "Right arrow\n"
    "Left arrow\n"
    "Down arrow\n"
    "Up arrow\n"
    "Page Up\n"
    "Double-click\n"
    "Right double-click\n"
    "Ctrl Right double-click\n"
    "Scroll, middle drag\n"
    "Left drag\n"
    "[Shift] right drag\n"
    "Ctrl [Shift] drag\n"
    "Ctrl [Shift] right drag";

const char help_title[] =
    "Swap left-right\n"
    "Show UI shortcuts\n"
    "Expand/collapse all  \n"
    "Pause\n"
    "Free camera\n"
    "Step forward\n"
    "Step back\n"
    "Step forward 100\n"
    "Step back 100\n"
    "Select parent\n"
    "Select\n"
    "Center\n"
    "Track camera\n"
    "Zoom\n"
    "View rotate\n"
    "View translate\n"
    "Object rotate\n"
    "Object translate";

// info strings
char info_title[1000];
char info_content[1000];

//----------------------- profiler, sensor, info, watch ---------------------------------

// init profiler figures
void profilerinit(void)
{
    int i, n;

    // set figures to default
    mjv_defaultFigure(&figconstraint);
    mjv_defaultFigure(&figcost);
    mjv_defaultFigure(&figtimer);
    mjv_defaultFigure(&figsize);

    // titles
    strcpy(figconstraint.title, "Counts");
    strcpy(figcost.title, "Convergence (log 10)");
    strcpy(figsize.title, "Dimensions");
    strcpy(figtimer.title, "CPU time (msec)");

    // x-labels
    strcpy(figconstraint.xlabel, "Solver iteration");
    strcpy(figcost.xlabel, "Solver iteration");
    strcpy(figsize.xlabel, "Video frame");
    strcpy(figtimer.xlabel, "Video frame");

    // y-tick nubmer formats
    strcpy(figconstraint.yformat, "%.0f");
    strcpy(figcost.yformat, "%.1f");
    strcpy(figsize.yformat, "%.0f");
    strcpy(figtimer.yformat, "%.2f");

    // colors
    figconstraint.figurergba[0] = 0.1f;
    figcost.figurergba[2] = 0.2f;
    figsize.figurergba[0] = 0.1f;
    figtimer.figurergba[2] = 0.2f;
    figconstraint.figurergba[3] = 0.5f;
    figcost.figurergba[3] = 0.5f;
    figsize.figurergba[3] = 0.5f;
    figtimer.figurergba[3] = 0.5f;

    // legends
    strcpy(figconstraint.linename[0], "total");
    strcpy(figconstraint.linename[1], "active");
    strcpy(figconstraint.linename[2], "changed");
    strcpy(figconstraint.linename[3], "evals");
    strcpy(figconstraint.linename[4], "updates");
    strcpy(figcost.linename[0], "improvement");
    strcpy(figcost.linename[1], "gradient");
    strcpy(figcost.linename[2], "lineslope");
    strcpy(figsize.linename[0], "dof");
    strcpy(figsize.linename[1], "body");
    strcpy(figsize.linename[2], "constraint");
    strcpy(figsize.linename[3], "sqrt(nnz)");
    strcpy(figsize.linename[4], "contact");
    strcpy(figsize.linename[5], "iteration");
    strcpy(figtimer.linename[0], "total");
    strcpy(figtimer.linename[1], "collision");
    strcpy(figtimer.linename[2], "prepare");
    strcpy(figtimer.linename[3], "solve");
    strcpy(figtimer.linename[4], "other");

    // grid sizes
    figconstraint.gridsize[0] = 5;
    figconstraint.gridsize[1] = 5;
    figcost.gridsize[0] = 5;
    figcost.gridsize[1] = 5;
    figsize.gridsize[0] = 3;
    figsize.gridsize[1] = 5;
    figtimer.gridsize[0] = 3;
    figtimer.gridsize[1] = 5;

    // minimum ranges
    figconstraint.range[0][0] = 0;
    figconstraint.range[0][1] = 20;
    figconstraint.range[1][0] = 0;
    figconstraint.range[1][1] = 80;
    figcost.range[0][0] = 0;
    figcost.range[0][1] = 20;
    figcost.range[1][0] = -15;
    figcost.range[1][1] = 5;
    figsize.range[0][0] = -200;
    figsize.range[0][1] = 0;
    figsize.range[1][0] = 0;
    figsize.range[1][1] = 100;
    figtimer.range[0][0] = -200;
    figtimer.range[0][1] = 0;
    figtimer.range[1][0] = 0;
    figtimer.range[1][1] = 0.4f;

    // init x axis on history figures (do not show yet)
    for (n = 0; n < 6; n++)
        for (i = 0; i < mjMAXLINEPNT; i++)
        {
            figtimer.linedata[n][2 * i] = (float)-i;
            figsize.linedata[n][2 * i] = (float)-i;
        }
}

// update profiler figures
void profilerupdate(void)
{
    int i, n;

    // update constraint figure
    figconstraint.linepnt[0] = mjMIN(mjMIN(d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
    for (i = 1; i < 5; i++)
        figconstraint.linepnt[i] = figconstraint.linepnt[0];
    if (m->opt.solver == mjSOL_PGS)
    {
        figconstraint.linepnt[3] = 0;
        figconstraint.linepnt[4] = 0;
    }
    if (m->opt.solver == mjSOL_CG)
        figconstraint.linepnt[4] = 0;
    for (i = 0; i < figconstraint.linepnt[0]; i++)
    {
        // x
        figconstraint.linedata[0][2 * i] = (float)i;
        figconstraint.linedata[1][2 * i] = (float)i;
        figconstraint.linedata[2][2 * i] = (float)i;
        figconstraint.linedata[3][2 * i] = (float)i;
        figconstraint.linedata[4][2 * i] = (float)i;

        // y
        figconstraint.linedata[0][2 * i + 1] = (float)d->nefc;
        figconstraint.linedata[1][2 * i + 1] = (float)d->solver[i].nactive;
        figconstraint.linedata[2][2 * i + 1] = (float)d->solver[i].nchange;
        figconstraint.linedata[3][2 * i + 1] = (float)d->solver[i].neval;
        figconstraint.linedata[4][2 * i + 1] = (float)d->solver[i].nupdate;
    }

    // update cost figure
    figcost.linepnt[0] = mjMIN(mjMIN(d->solver_iter, mjNSOLVER), mjMAXLINEPNT);
    for (i = 1; i < 3; i++)
        figcost.linepnt[i] = figcost.linepnt[0];
    if (m->opt.solver == mjSOL_PGS)
    {
        figcost.linepnt[1] = 0;
        figcost.linepnt[2] = 0;
    }

    for (i = 0; i < figcost.linepnt[0]; i++)
    {
        // x
        figcost.linedata[0][2 * i] = (float)i;
        figcost.linedata[1][2 * i] = (float)i;
        figcost.linedata[2][2 * i] = (float)i;

        // y
        figcost.linedata[0][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].improvement));
        figcost.linedata[1][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].gradient));
        figcost.linedata[2][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, d->solver[i].lineslope));
    }

    // get timers: total, collision, prepare, solve, other
    mjtNum total = d->timer[mjTIMER_STEP].duration;
    int number = d->timer[mjTIMER_STEP].number;
    if (!number)
    {
        total = d->timer[mjTIMER_FORWARD].duration;
        number = d->timer[mjTIMER_FORWARD].number;
    }
    number = mjMAX(1, number);
    float tdata[5] = {
        (float)(total / number),
        (float)(d->timer[mjTIMER_POS_COLLISION].duration / number),
        (float)(d->timer[mjTIMER_POS_MAKE].duration / number) +
            (float)(d->timer[mjTIMER_POS_PROJECT].duration / number),
        (float)(d->timer[mjTIMER_CONSTRAINT].duration / number),
        0};
    tdata[4] = tdata[0] - tdata[1] - tdata[2] - tdata[3];

    // update figtimer
    int pnt = mjMIN(201, figtimer.linepnt[0] + 1);
    for (n = 0; n < 5; n++)
    {
        // shift data
        for (i = pnt - 1; i > 0; i--)
            figtimer.linedata[n][2 * i + 1] = figtimer.linedata[n][2 * i - 1];

        // assign new
        figtimer.linepnt[n] = pnt;
        figtimer.linedata[n][1] = tdata[n];
    }

    // get sizes: nv, nbody, nefc, sqrt(nnz), ncont, iter
    float sdata[6] = {
        (float)m->nv,
        (float)m->nbody,
        (float)d->nefc,
        (float)mju_sqrt((mjtNum)d->solver_nnz),
        (float)d->ncon,
        (float)d->solver_iter};

    // update figsize
    pnt = mjMIN(201, figsize.linepnt[0] + 1);
    for (n = 0; n < 6; n++)
    {
        // shift data
        for (i = pnt - 1; i > 0; i--)
            figsize.linedata[n][2 * i + 1] = figsize.linedata[n][2 * i - 1];

        // assign new
        figsize.linepnt[n] = pnt;
        figsize.linedata[n][1] = sdata[n];
    }
}

// show profiler figures
void profilershow(mjrRect rect)
{
    mjrRect viewport = {
        rect.left + rect.width - rect.width / 4,
        rect.bottom,
        rect.width / 4,
        rect.height / 4};
    mjr_figure(viewport, &figtimer, &con);
    viewport.bottom += rect.height / 4;
    mjr_figure(viewport, &figsize, &con);
    viewport.bottom += rect.height / 4;
    mjr_figure(viewport, &figcost, &con);
    viewport.bottom += rect.height / 4;
    mjr_figure(viewport, &figconstraint, &con);
}

// init sensor figure
void sensorinit(void)
{
    // set figure to default
    mjv_defaultFigure(&figstate);
    figstate.figurergba[3] = 0.3f;
    figstate.legendrgba[3] = 0;
    figstate.gridsize[0] = 5;
    figstate.gridsize[1] = 5;
    // minimum range
    figstate.range[0][0] = 1;
    figstate.range[0][1] = 100;
    figstate.range[1][0] = 0;
    figstate.range[1][1] = 1;
    strcpy(figstate.title, "Planner State");
    strcpy(figstate.xlabel, "Iterations");
    strcpy(figstate.yformat, "%.2f");
    figstate.panergba[3] = 0.3f;
    strcpy(figstate.linename[0], "Foothold");
    strcpy(figstate.linename[1], "Swing");

    mjv_defaultFigure(&figtimecost);
    figtimecost.figurergba[3] = 0.3f;
    figtimecost.legendrgba[3] = 0;
    figtimecost.gridsize[0] = 5;
    figtimecost.gridsize[1] = 5;
    // minimum range
    figtimecost.range[0][0] = 1;
    figtimecost.range[0][1] = 100;
    figtimecost.range[1][0] = 0;
    figtimecost.range[1][1] = 0.1;
    strcpy(figtimecost.title, "Planner Time Cost");
    strcpy(figtimecost.xlabel, "Iterations");
    strcpy(figtimecost.yformat, "%.2f");
    figtimecost.panergba[3] = 0.3f;
    strcpy(figtimecost.linename[0], "Foot Planner");

    mjv_defaultFigure(&figvelocity);
    figvelocity.figurergba[3] = 0.3f;
    figvelocity.legendrgba[3] = 0;
    figvelocity.gridsize[0] = 5;
    figvelocity.gridsize[1] = 5;
    // minimum range
    figvelocity.range[0][0] = 1;
    figvelocity.range[0][1] = 100;
    figvelocity.range[1][0] = 0;
    figvelocity.range[1][1] = 1;
    strcpy(figvelocity.title, "Velocity data");
    strcpy(figvelocity.xlabel, "Iterations");
    strcpy(figvelocity.yformat, "%.2f");
    figvelocity.panergba[3] = 0.3f;
    strcpy(figvelocity.linename[0], "CMD");
    strcpy(figvelocity.linename[1], "Foot Planed");
    strcpy(figvelocity.linename[2], "Real");
}

// update sensor figure
void sensorupdate(void)
{
    figstate.linepnt[0] = 100;
    figstate.linepnt[1] = 100;
    // shift data
    for (int i = 100 - 1; i > 0; i--)
    {
        figstate.linedata[0][2 * i + 1] = figstate.linedata[0][2 * i - 1];
        figstate.linedata[1][2 * i + 1] = figstate.linedata[1][2 * i - 1];
    }
    for (int i = 0; i < 100; i++)
    {
        figstate.linedata[0][2 * i] = i + 1;
        figstate.linedata[1][2 * i] = i + 1;
    }
    // assign new
    figstate.linedata[0][1] = foothold_planner_state ? 1.0 : 0;
    figstate.linedata[1][1] = swing_planner_state;

    figtimecost.linepnt[0] = 100;
    // shift data
    for (int i = 100 - 1; i > 0; i--)
    {
        figtimecost.linedata[0][2 * i + 1] = figtimecost.linedata[0][2 * i - 1];
    }
    for (int i = 0; i < 100; i++)
    {
        figtimecost.linedata[0][2 * i] = i + 1;
    }
    // assign new
    figtimecost.linedata[0][1] = foot_planner_time;

    figvelocity.linepnt[0] = 100;
    figvelocity.linepnt[1] = 100;
    figvelocity.linepnt[2] = 100;
    // shift data
    for (int i = 100 - 1; i > 0; i--)
    {
        figvelocity.linedata[0][2 * i + 1] = figvelocity.linedata[0][2 * i - 1];
        figvelocity.linedata[1][2 * i + 1] = figvelocity.linedata[1][2 * i - 1];
        figvelocity.linedata[2][2 * i + 1] = figvelocity.linedata[2][2 * i - 1];
    }
    for (int i = 0; i < 100; i++)
    {
        figvelocity.linedata[0][2 * i] = i + 1;
        figvelocity.linedata[1][2 * i] = i + 1;
        figvelocity.linedata[2][2 * i] = i + 1;
    }

    // assign new
    figvelocity.linedata[0][20 + 1] = cmd_velocity;
    figvelocity.linedata[1][20 + 1] = planned_velocity[0];
    for (int i = 0; i < 5; i++)
    {
        figvelocity.linedata[1][10 + 2 * i + 1] = planned_velocity[1];
        figvelocity.linedata[1][2 * i + 1] = planned_velocity[2];
    }
    figvelocity.linedata[2][20 + 1] = vbody_real[0];
}

// show sensor figure
void sensorshow(mjrRect rect)
{
    // constant width with and without profiler
    int width = settings.profiler ? rect.width / 3 : rect.width / 4;

    // render figure on the right
    mjrRect viewport = {
        rect.left + rect.width - width,
        rect.bottom,
        width,
        rect.height / 4};
    mjr_figure(viewport, &figstate, &con);
    viewport.bottom += rect.height / 4;
    mjr_figure(viewport, &figtimecost, &con);
    viewport.bottom += rect.height / 4;
    mjr_figure(viewport, &figvelocity, &con);
}

// prepare info text
void infotext(char *title, char *content, double interval)
{
    char tmp[20];

    // compute solver error
    mjtNum solerr = 0;
    if (d->solver_iter)
    {
        int ind = mjMIN(d->solver_iter - 1, mjNSOLVER - 1);
        solerr = mju_min(d->solver[ind].improvement, d->solver[ind].gradient);
        if (solerr == 0)
            solerr = mju_max(d->solver[ind].improvement, d->solver[ind].gradient);
    }
    solerr = mju_log10(mju_max(mjMINVAL, solerr));

    // prepare info text
    strcpy(title, "Time\nSize\nCPU\nSolver   \nFPS\nstack\nconbuf\nefcbuf");
    sprintf(content, "%-20.3f\n%d  (%d con)\n%.3f\n%.1f  (%d it)\n%.0f\n%.3f\n%.3f\n%.3f",
            d->time,
            d->nefc, d->ncon,
            settings.run ? d->timer[mjTIMER_STEP].duration / mjMAX(1, d->timer[mjTIMER_STEP].number) : d->timer[mjTIMER_FORWARD].duration / mjMAX(1, d->timer[mjTIMER_FORWARD].number),
            solerr, d->solver_iter,
            1 / interval,
            d->maxuse_stack / (double)d->nstack,
            d->maxuse_con / (double)m->nconmax,
            d->maxuse_efc / (double)m->njmax);

    // add Energy if enabled
    if (mjENABLED(mjENBL_ENERGY))
    {
        sprintf(tmp, "\n%.3f", d->energy[0] + d->energy[1]);
        strcat(content, tmp);
        strcat(title, "\nEnergy");
    }

    // add FwdInv if enabled
    if (mjENABLED(mjENBL_FWDINV))
    {
        sprintf(tmp, "\n%.1f %.1f",
                mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[0])),
                mju_log10(mju_max(mjMINVAL, d->solver_fwdinv[1])));
        strcat(content, tmp);
        strcat(title, "\nFwdInv");
    }
}

// sprintf forwarding, to avoid compiler warning in x-macro
void printfield(char *str, void *ptr)
{
    sprintf(str, "%g", *(mjtNum *)ptr);
}

// update watch
void watch(void)
{
    // clear
    ui0.sect[SECT_WATCH].item[2].multi.nelem = 1;
    strcpy(ui0.sect[SECT_WATCH].item[2].multi.name[0], "invalid field");

    // prepare constants for NC
    int nv = m->nv;
    int njmax = m->njmax;

    // find specified field in mjData arrays, update value
#define X(TYPE, NAME, NR, NC)                                       \
    if (!strcmp(#NAME, settings.field) && !strcmp(#TYPE, "mjtNum")) \
    {                                                               \
        if (settings.index >= 0 && settings.index < m->NR * NC)     \
            printfield(ui0.sect[SECT_WATCH].item[2].multi.name[0],  \
                       d->NAME + settings.index);                   \
        else                                                        \
            strcpy(ui0.sect[SECT_WATCH].item[2].multi.name[0],      \
                   "invalid index");                                \
        return;                                                     \
    }

    MJDATA_POINTERS
#undef X
}

//-------------------------------- UI construction --------------------------------------

// make physics section of UI
void makephysics(int oldstate)
{
    int i;

    mjuiDef defPhysics[] =
        {
            {mjITEM_SECTION, "Physics", oldstate, NULL, "AP"},
            {mjITEM_SELECT, "Integrator", 2, &(m->opt.integrator), "Euler\nRK4"},
            {mjITEM_SELECT, "Collision", 2, &(m->opt.collision), "All\nPair\nDynamic"},
            {mjITEM_SELECT, "Cone", 2, &(m->opt.cone), "Pyramidal\nElliptic"},
            {mjITEM_SELECT, "Jacobian", 2, &(m->opt.jacobian), "Dense\nSparse\nAuto"},
            {mjITEM_SELECT, "Solver", 2, &(m->opt.solver), "PGS\nCG\nNewton"},
            {mjITEM_SEPARATOR, "Algorithmic Parameters", 1},
            {mjITEM_EDITNUM, "Timestep", 2, &(m->opt.timestep), "1 0 1"},
            {mjITEM_EDITINT, "Iterations", 2, &(m->opt.iterations), "1 0 1000"},
            {mjITEM_EDITNUM, "Tolerance", 2, &(m->opt.tolerance), "1 0 1"},
            {mjITEM_EDITINT, "Noslip Iter", 2, &(m->opt.noslip_iterations), "1 0 1000"},
            {mjITEM_EDITNUM, "Noslip Tol", 2, &(m->opt.noslip_tolerance), "1 0 1"},
            {mjITEM_EDITINT, "MRR Iter", 2, &(m->opt.mpr_iterations), "1 0 1000"},
            {mjITEM_EDITNUM, "MPR Tol", 2, &(m->opt.mpr_tolerance), "1 0 1"},
            {mjITEM_EDITNUM, "API Rate", 2, &(m->opt.apirate), "1 0 1000"},
            {mjITEM_SEPARATOR, "Physical Parameters", 1},
            {mjITEM_EDITNUM, "Gravity", 2, m->opt.gravity, "3"},
            {mjITEM_EDITNUM, "Wind", 2, m->opt.wind, "3"},
            {mjITEM_EDITNUM, "Magnetic", 2, m->opt.magnetic, "3"},
            {mjITEM_EDITNUM, "Density", 2, &(m->opt.density), "1"},
            {mjITEM_EDITNUM, "Viscosity", 2, &(m->opt.viscosity), "1"},
            {mjITEM_EDITNUM, "Imp Ratio", 2, &(m->opt.impratio), "1"},
            {mjITEM_SEPARATOR, "Disable Flags", 1},
            {mjITEM_END}};
    mjuiDef defEnableFlags[] =
        {
            {mjITEM_SEPARATOR, "Enable Flags", 1},
            {mjITEM_END}};
    mjuiDef defOverride[] =
        {
            {mjITEM_SEPARATOR, "Contact Override", 1},
            {mjITEM_EDITNUM, "Margin", 2, &(m->opt.o_margin), "1"},
            {mjITEM_EDITNUM, "Sol Imp", 2, &(m->opt.o_solimp), "5"},
            {mjITEM_EDITNUM, "Sol Ref", 2, &(m->opt.o_solref), "2"},
            {mjITEM_END}};

    // add physics
    mjui_add(&ui0, defPhysics);

    // add flags programmatically
    mjuiDef defFlag[] =
        {
            {mjITEM_CHECKINT, "", 2, NULL, ""},
            {mjITEM_END}};
    for (i = 0; i < mjNDISABLE; i++)
    {
        strcpy(defFlag[0].name, mjDISABLESTRING[i]);
        defFlag[0].pdata = settings.disable + i;
        mjui_add(&ui0, defFlag);
    }
    mjui_add(&ui0, defEnableFlags);
    for (i = 0; i < mjNENABLE; i++)
    {
        strcpy(defFlag[0].name, mjENABLESTRING[i]);
        defFlag[0].pdata = settings.enable + i;
        mjui_add(&ui0, defFlag);
    }

    // add contact override
    mjui_add(&ui0, defOverride);
}

// make rendering section of UI
void makerendering(int oldstate)
{
    int i, j;

    mjuiDef defRendering[] =
        {
            {mjITEM_SECTION, "Rendering", oldstate, NULL, "AR"},
            {mjITEM_SELECT, "Camera", 2, &(settings.camera), "Free\nTracking"},
            {mjITEM_SELECT, "Label", 2, &(vopt.label),
             "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\nActuator\nConstraint\nSkin\nSelection\nSel Pnt\nForce"},
            {mjITEM_SELECT, "Frame", 2, &(vopt.frame),
             "None\nBody\nGeom\nSite\nCamera\nLight\nWorld"},
            {mjITEM_SEPARATOR, "Model Elements", 1},
            {mjITEM_END}};
    mjuiDef defOpenGL[] =
        {
            {mjITEM_SEPARATOR, "OpenGL Effects", 1},
            {mjITEM_END}};

    // add model cameras, up to UI limit
    for (i = 0; i < mjMIN(m->ncam, mjMAXUIMULTI - 2); i++)
    {
        // prepare name
        char camname[mjMAXUITEXT] = "\n";
        if (m->names[m->name_camadr[i]])
            strcat(camname, m->names + m->name_camadr[i]);
        else
            sprintf(camname, "\nCamera %d", i);

        // check string length
        if (strlen(camname) + strlen(defRendering[1].other) >= mjMAXUITEXT - 1)
            break;

        // add camera
        strcat(defRendering[1].other, camname);
    }

    // add rendering standard
    mjui_add(&ui0, defRendering);

    // add flags programmatically
    mjuiDef defFlag[] =
        {
            {mjITEM_CHECKBYTE, "", 2, NULL, ""},
            {mjITEM_END}};
    for (i = 0; i < mjNVISFLAG; i++)
    {
        // set name, remove "&"
        strcpy(defFlag[0].name, mjVISSTRING[i][0]);
        for (j = 0; j < strlen(mjVISSTRING[i][0]); j++)
            if (mjVISSTRING[i][0][j] == '&')
            {
                strcpy(defFlag[0].name + j, mjVISSTRING[i][0] + j + 1);
                break;
            }

        // set shortcut and data
        sprintf(defFlag[0].other, " %s", mjVISSTRING[i][2]);
        defFlag[0].pdata = vopt.flags + i;
        mjui_add(&ui0, defFlag);
    }
    mjui_add(&ui0, defOpenGL);
    for (i = 0; i < mjNRNDFLAG; i++)
    {
        strcpy(defFlag[0].name, mjRNDSTRING[i][0]);
        sprintf(defFlag[0].other, " %s", mjRNDSTRING[i][2]);
        defFlag[0].pdata = scn.flags + i;
        mjui_add(&ui0, defFlag);
    }
}

// make group section of UI
void makegroup(int oldstate)
{
    mjuiDef defGroup[] =
        {
            {mjITEM_SECTION, "Group enable", oldstate, NULL, "AG"},
            {mjITEM_SEPARATOR, "Geom groups", 1},
            {mjITEM_CHECKBYTE, "Geom 0", 2, vopt.geomgroup, " 0"},
            {mjITEM_CHECKBYTE, "Geom 1", 2, vopt.geomgroup + 1, " 1"},
            {mjITEM_CHECKBYTE, "Geom 2", 2, vopt.geomgroup + 2, " 2"},
            {mjITEM_CHECKBYTE, "Geom 3", 2, vopt.geomgroup + 3, " 3"},
            {mjITEM_CHECKBYTE, "Geom 4", 2, vopt.geomgroup + 4, " 4"},
            {mjITEM_CHECKBYTE, "Geom 5", 2, vopt.geomgroup + 5, " 5"},
            {mjITEM_SEPARATOR, "Site groups", 1},
            {mjITEM_CHECKBYTE, "Site 0", 2, vopt.sitegroup, "S0"},
            {mjITEM_CHECKBYTE, "Site 1", 2, vopt.sitegroup + 1, "S1"},
            {mjITEM_CHECKBYTE, "Site 2", 2, vopt.sitegroup + 2, "S2"},
            {mjITEM_CHECKBYTE, "Site 3", 2, vopt.sitegroup + 3, "S3"},
            {mjITEM_CHECKBYTE, "Site 4", 2, vopt.sitegroup + 4, "S4"},
            {mjITEM_CHECKBYTE, "Site 5", 2, vopt.sitegroup + 5, "S5"},
            {mjITEM_SEPARATOR, "Joint groups", 1},
            {mjITEM_CHECKBYTE, "Joint 0", 2, vopt.jointgroup, ""},
            {mjITEM_CHECKBYTE, "Joint 1", 2, vopt.jointgroup + 1, ""},
            {mjITEM_CHECKBYTE, "Joint 2", 2, vopt.jointgroup + 2, ""},
            {mjITEM_CHECKBYTE, "Joint 3", 2, vopt.jointgroup + 3, ""},
            {mjITEM_CHECKBYTE, "Joint 4", 2, vopt.jointgroup + 4, ""},
            {mjITEM_CHECKBYTE, "Joint 5", 2, vopt.jointgroup + 5, ""},
            {mjITEM_SEPARATOR, "Tendon groups", 1},
            {mjITEM_CHECKBYTE, "Tendon 0", 2, vopt.tendongroup, ""},
            {mjITEM_CHECKBYTE, "Tendon 1", 2, vopt.tendongroup + 1, ""},
            {mjITEM_CHECKBYTE, "Tendon 2", 2, vopt.tendongroup + 2, ""},
            {mjITEM_CHECKBYTE, "Tendon 3", 2, vopt.tendongroup + 3, ""},
            {mjITEM_CHECKBYTE, "Tendon 4", 2, vopt.tendongroup + 4, ""},
            {mjITEM_CHECKBYTE, "Tendon 5", 2, vopt.tendongroup + 5, ""},
            {mjITEM_SEPARATOR, "Actuator groups", 1},
            {mjITEM_CHECKBYTE, "Actuator 0", 2, vopt.actuatorgroup, ""},
            {mjITEM_CHECKBYTE, "Actuator 1", 2, vopt.actuatorgroup + 1, ""},
            {mjITEM_CHECKBYTE, "Actuator 2", 2, vopt.actuatorgroup + 2, ""},
            {mjITEM_CHECKBYTE, "Actuator 3", 2, vopt.actuatorgroup + 3, ""},
            {mjITEM_CHECKBYTE, "Actuator 4", 2, vopt.actuatorgroup + 4, ""},
            {mjITEM_CHECKBYTE, "Actuator 5", 2, vopt.actuatorgroup + 5, ""},
            {mjITEM_END}};

    // add section
    mjui_add(&ui0, defGroup);
}

// make joint section of UI
void makejoint(int oldstate)
{
    int i;

    mjuiDef defJoint[] =
        {
            {mjITEM_SECTION, "Joint", oldstate, NULL, "AJ"},
            {mjITEM_END}};
    mjuiDef defSlider[] =
        {
            {mjITEM_SLIDERNUM, "", 2, NULL, "0 1"},
            {mjITEM_END}};

    // add section
    mjui_add(&ui1, defJoint);
    defSlider[0].state = 4;

    // add scalar joints, exit if UI limit reached
    int itemcnt = 0;
    for (i = 0; i < m->njnt && itemcnt < mjMAXUIITEM; i++)
        if ((m->jnt_type[i] == mjJNT_HINGE || m->jnt_type[i] == mjJNT_SLIDE))
        {
            // skip if joint group is disabled
            if (!vopt.jointgroup[mjMAX(0, mjMIN(mjNGROUP - 1, m->jnt_group[i]))])
                continue;

            // set data and name
            defSlider[0].pdata = d->qpos + m->jnt_qposadr[i];
            if (m->names[m->name_jntadr[i]])
                mju_strncpy(defSlider[0].name, m->names + m->name_jntadr[i],
                            mjMAXUINAME);
            else
                sprintf(defSlider[0].name, "joint %d", i);

            // set range
            if (m->jnt_limited[i])
                sprintf(defSlider[0].other, "%.4g %.4g",
                        m->jnt_range[2 * i], m->jnt_range[2 * i + 1]);
            else if (m->jnt_type[i] == mjJNT_SLIDE)
                strcpy(defSlider[0].other, "-1 1");
            else
                strcpy(defSlider[0].other, "-3.1416 3.1416");

            // add and count
            mjui_add(&ui1, defSlider);
            itemcnt++;
        }
}

// make control section of UI
void makecontrol(int oldstate)
{
    int i;

    mjuiDef defControl[] =
        {
            {mjITEM_SECTION, "Control", oldstate, NULL, "AC"},
            {mjITEM_BUTTON, "Clear all", 2},
            {mjITEM_END}};
    mjuiDef defSlider[] =
        {
            {mjITEM_SLIDERNUM, "", 2, NULL, "0 1"},
            {mjITEM_END}};

    // add section
    mjui_add(&ui1, defControl);
    defSlider[0].state = 2;

    // add controls, exit if UI limit reached (Clear button already added)
    int itemcnt = 1;
    for (i = 0; i < m->nu && itemcnt < mjMAXUIITEM; i++)
    {
        // skip if actuator group is disabled
        if (!vopt.actuatorgroup[mjMAX(0, mjMIN(mjNGROUP - 1, m->actuator_group[i]))])
            continue;

        // set data and name
        defSlider[0].pdata = d->ctrl + i;
        if (m->names[m->name_actuatoradr[i]])
            mju_strncpy(defSlider[0].name, m->names + m->name_actuatoradr[i],
                        mjMAXUINAME);
        else
            sprintf(defSlider[0].name, "control %d", i);

        // set range
        if (m->actuator_ctrllimited[i])
            sprintf(defSlider[0].other, "%.4g %.4g",
                    m->actuator_ctrlrange[2 * i], m->actuator_ctrlrange[2 * i + 1]);
        else
            strcpy(defSlider[0].other, "-1 1");

        // add and count
        mjui_add(&ui1, defSlider);
        itemcnt++;
    }
}

// make model-dependent UI sections
void makesections(void)
{
    int i;

    // get section open-close state, UI 0
    int oldstate0[NSECT0];
    for (i = 0; i < NSECT0; i++)
    {
        oldstate0[i] = 0;
        if (ui0.nsect > i)
            oldstate0[i] = ui0.sect[i].state;
    }

    // get section open-close state, UI 1
    int oldstate1[NSECT1];
    for (i = 0; i < NSECT1; i++)
    {
        oldstate1[i] = 0;
        if (ui1.nsect > i)
            oldstate1[i] = ui1.sect[i].state;
    }

    // clear model-dependent sections of UI
    ui0.nsect = SECT_PHYSICS;
    ui1.nsect = 0;

    // make
    makephysics(oldstate0[SECT_PHYSICS]);
    makerendering(oldstate0[SECT_RENDERING]);
    makegroup(oldstate0[SECT_GROUP]);
    makejoint(oldstate1[SECT_JOINT]);
    makecontrol(oldstate1[SECT_CONTROL]);
}

//-------------------------------- utility functions ------------------------------------

// align and scale view
void alignscale(void)
{
    // autoscale
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;

    // set to free camera
    //    cam.type = mjCAMERA_FREE;
    cam.type = mjCAMERA_TRACKING;
    cam.trackbodyid = 1;
    cam.fixedcamid = -1;
}

// copy qpos to clipboard as key
void copykey(void)
{
    char clipboard[5000] = "<key qpos='";
    char buf[200];

    // prepare string
    for (int i = 0; i < m->nq; i++)
    {
        sprintf(buf, i == m->nq - 1 ? "%g" : "%g ", d->qpos[i]);
        strcat(clipboard, buf);
    }
    strcat(clipboard, "'/>");

    // copy to clipboard
    glfwSetClipboardString(window, clipboard);
}

// millisecond timer, for MuJoCo built-in profiler
mjtNum timer(void)
{
    return (mjtNum)(1000 * glfwGetTime());
}

// clear all times
void cleartimers(void)
{
    for (int i = 0; i < mjNTIMER; i++)
    {
        d->timer[i].duration = 0;
        d->timer[i].number = 0;
    }
}

// update UI 0 when MuJoCo structures change (except for joint sliders)
void updatesettings(void)
{
    int i;

    // physics flags
    for (i = 0; i < mjNDISABLE; i++)
        settings.disable[i] = ((m->opt.disableflags & (1 << i)) != 0);
    for (i = 0; i < mjNENABLE; i++)
        settings.enable[i] = ((m->opt.enableflags & (1 << i)) != 0);

    // camera
    if (cam.type == mjCAMERA_FIXED)
        settings.camera = 2 + cam.fixedcamid;
    else if (cam.type == mjCAMERA_TRACKING)
        settings.camera = 1;
    else
        settings.camera = 0;

    // update UI
    mjui_update(-1, -1, &ui0, &uistate, &con);
}

// drop file callback
void drop(GLFWwindow *window, int count, const char **paths)
{
    // make sure list is non-empty
    if (count > 0)
    {
        mju_strncpy(filename, paths[0], 1000);
        settings.loadrequest = 1;
    }
}

int updateHfield_needed = false;
int shift = 0;
void updateHfield()
{
    if (updateHfield_needed)
    {
        // m->geom_pos[3] = d->qpos[0];
        // m->geom_pos[4] = d->qpos[1];
        if (shift % 100 == 0)
        {

            updateHfield_needed = false;
            memset(m->hfield_data, 0, m->hfield_ncol[0] * m->hfield_nrow[0] * sizeof(double));
            /* double stair_d = 0.3;
            double stair_l = 1.;
            double stair_h = 0.15;
            for (int i = 0; i < 12; i++)
            {
                Plane one_plane;
                if (i < 6)
                {
                    one_plane.A << 0, 0, stair_h * (1 + i);
                }
                else if (i > 6)
                {
                    one_plane.A << 0, 0, stair_h * (13 - i);
                }
                else
                {
                    one_plane.A << 0, 0, stair_h * (1 + 5);
                }
                one_plane.bound_num = 4;
                one_plane.Bs[0] = Eigen::Vector3d(1, 0, 0.5 + (i + 1) * stair_d - 0);
                one_plane.Bs[1] = Eigen::Vector3d(0, -1, stair_l / 2);
                one_plane.Bs[2] = Eigen::Vector3d(-1, 0, -(0.5 + i * stair_d));
                one_plane.Bs[3] = Eigen::Vector3d(0, 1, stair_l / 2);
                perception_planes[i] = one_plane;
            }
            // planes[7].A << 0, 0, stair_h*(1+5);
            // planes[8].A << 0, 0, stair_h*(1+5);

            perception_planes[12].A << 0, 0, 0;
            perception_planes[12].bound_num = 4;
            perception_planes[12].Bs[0] = Eigen::Vector3d(0, 1, stair_l / 2);
            perception_planes[12].Bs[1] = Eigen::Vector3d(1, 0, 0.5);
            perception_planes[12].Bs[2] = Eigen::Vector3d(0, -1, stair_l / 2);
            perception_planes[12].Bs[3] = Eigen::Vector3d(-1, 0, 5);

            Eigen::Matrix2d Bounds;
            Vec2 uB;
            for (int n = 0; n < perception_planes.size(); n++)
            {
                if (perception_planes[n].bound_num > 0)
                {
                    for (int i = 0; i < perception_planes[n].bound_num; i++)
                    {
                        Bounds.row(0) = perception_planes[n].Bs[i].head(2);
                        Bounds.row(1) = perception_planes[n].Bs[(i + 1) % perception_planes[n].bound_num].head(2);
                        uB << perception_planes[n].Bs[i][2], perception_planes[n].Bs[(i + 1) % perception_planes[n].bound_num][2];
                        perception_planes[n].Vertexs[i] = Bounds.inverse() * uB;
                        // std::cout << "plane: " << n << std::endl << planes[n].Vertexs[i] << std::endl;
                    }
                }
            }*/

            /* for (int i = 0; i < 3; i++)
            {
                printf("bound_num: %d\n", perception_planes[i].bound_num);
                std::cout << "A: " << perception_planes[i].A.transpose() << std::endl;
                for (int n = 0; n < 6; n++)
                {
                    std::cout << "(" << i << "," << n << ")"
                            << " Bs: " << perception_planes[i].Bs[n].transpose() << std::endl;
                    std::cout << "(" << i << "," << n << ")"
                            << " Vertex: " << perception_planes[i].Vertexs[n].transpose() << std::endl;
                }
            } */

            // perception_planes[0].bound_num = 4;
            // perception_planes[0].A << 0, 0, 0;
            // perception_planes[0].Bs[0] = Eigen::Vector3d(0, 1, 4.0);
            // perception_planes[0].Bs[1] = Eigen::Vector3d(1, 0, 5);
            // perception_planes[0].Bs[2] = Eigen::Vector3d(0, -1, 4.0);
            // perception_planes[0].Bs[3] = Eigen::Vector3d(-1, 0, 5);

            double radius_x = m->hfield_size[0];
            double radius_y = m->hfield_size[1];
            double pos_x = m->geom_pos[3];
            double pos_y = m->geom_pos[4];
            double max_z = m->hfield_size[2];
            // assign hfield, added by syx
            for (int i = 0; i < m->hfield_ncol[0]; i++)
            {
                double tmp_x = pos_x - radius_x + i * 2 * radius_x / m->hfield_ncol[0];
                for (int p_c = 0; p_c < plane_num; p_c++)
                {
                    auto &one_plane = perception_planes[p_c];
                    if (one_plane.bound_num == 0)
                        continue;
                    double max_y = pos_y + radius_y;
                    double min_y = pos_y - radius_y;
                    for (int i = 0; i < one_plane.bound_num; i++)
                    {
                        auto B = one_plane.Bs[i];
                        if (B[1] > 0)
                        {
                            double tmp_y = ((B[2] - B[0] * tmp_x) / B[1]);
                            max_y = std::min(tmp_y, max_y);
                        }
                        else if (B[1] < 0)
                        {
                            double tmp_y = ((B[2] - B[0] * tmp_x) / B[1]);
                            min_y = std::max(tmp_y, min_y);
                        }
                        else if (B[1] == 0)
                        {
                            if (B[0] * tmp_x > B[2])
                            {
                                min_y = max_y;
                            }
                        }
                    }
                    int lower_j = std::max(int((min_y - (pos_y - radius_y)) * m->hfield_nrow[0] / (2 * radius_y)), 0);
                    int upper_j = std::min(int((max_y - (pos_y - radius_y)) * m->hfield_nrow[0] / (2 * radius_y)), m->hfield_nrow[0]);
                    for (int j = lower_j; j < upper_j; j++)
                    {
                        double tmp_y = pos_y - radius_y + j * (2 * radius_y) / m->hfield_nrow[0];
                        double tmp_z = one_plane.A[0] * tmp_x + one_plane.A[1] * tmp_y + one_plane.A[2];
                        double height = std::min(tmp_z, max_z) / max_z;
                        m->hfield_data[m->hfield_ncol[0] * j + i] = height;
                        // m->hfield_data[m->hfield_ncol[0] * j + i] = (0.2 * sin(j * 0.2 + shift) + 0.2) / max_z;
                        // if (j == lower_j)
                        // {
                        //     std::cout << "i: " << i << " j: " << j << " height: " << tmp_z << std::endl;
                        // }
                    }
                }
                // for (int j=0; j<m->hfield_nrow[0]; j++) {
                //     m->hfield_data[m->hfield_ncol[0]*j+i] = (float)i/m->hfield_ncol[0];
                // }
            }
            mjr_uploadHField(m, &con, 0);
        }
        // printf("finish generate height!");
        shift++;
    }
}

// load mjb or xml model
void loadmodel(void)
{
    // clear request
    settings.loadrequest = 0;

    // make sure filename is not empty
    if (!filename[0])
        return;

    // load and compile
    char error[500] = "";
    mjModel *mnew = 0;
    if (strlen(filename) > 4 && !strcmp(filename + strlen(filename) - 4, ".mjb"))
    {
        mnew = mj_loadModel(filename, NULL);
        if (!mnew)
            strcpy(error, "could not load binary model");
    }
    else
        mnew = mj_loadXML(filename, NULL, error, 500);
    if (!mnew)
    {
        printf("%s\n", error);
        return;
    }

    // compiler warning: print and pause
    if (error[0])
    {
        // mj_forward() below will print the warning message
        printf("Model compiled, but simulation warning (paused):\n  %s\n\n",
               error);
        settings.run = 0;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);
    mj_forward(m, d);

    // re-create scene and context
    mjv_makeScene(m, &scn, maxgeom);
    mjr_makeContext(m, &con, 50 * (settings.font + 1));

    // clear perturbation state
    pert.active = 0;
    pert.select = 0;
    pert.skinselect = -1;

    // align and scale view, update scene
    alignscale();
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // set window title to model name
    if (window && m->names)
    {
        char title[200] = "state visualizer : ";
        strcat(title, m->names);
        glfwSetWindowTitle(window, title);
    }

    // set keyframe range and divisions
    ui0.sect[SECT_SIMULATION].item[5].slider.range[0] = 0;
    ui0.sect[SECT_SIMULATION].item[5].slider.range[1] = mjMAX(0, m->nkey - 1);
    ui0.sect[SECT_SIMULATION].item[5].slider.divisions = mjMAX(1, m->nkey - 1);

    // rebuild UI sections
    makesections();

    // full ui update
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);
    updatesettings();

#ifdef SET_INIT_QPOS
    memcpy(d->qpos, q_init, m->nq * sizeof(mjtNum));
#endif
}

//--------------------------------- UI hooks (for uitools.c) ----------------------------

// determine enable/disable item state given category
int uiPredicate(int category, void *userdata)
{
    switch (category)
    {
    case 2: // require model
        return (m != NULL);

    case 3: // require model and nkey
        return (m && m->nkey);

    case 4: // require model and paused
        return (m && !settings.run);

    default:
        return 1;
    }
}

// set window layout
void uiLayout(mjuiState *state)
{
    mjrRect *rect = state->rect;

    // set number of rectangles
    state->nrect = 4;

    // rect 0: entire framebuffer
    rect[0].left = 0;
    rect[0].bottom = 0;
    glfwGetFramebufferSize(window, &rect[0].width, &rect[0].height);

    // rect 1: UI 0
    rect[1].left = 0;
    rect[1].width = settings.ui0 ? ui0.width : 0;
    rect[1].bottom = 0;
    rect[1].height = rect[0].height;

    // rect 2: UI 1
    rect[2].width = settings.ui1 ? ui1.width : 0;
    rect[2].left = mjMAX(0, rect[0].width - rect[2].width);
    rect[2].bottom = 0;
    rect[2].height = rect[0].height;

    // rect 3: 3D plot (everything else is an overlay)
    rect[3].left = rect[1].width;
    rect[3].width = mjMAX(0, rect[0].width - rect[1].width - rect[2].width);
    rect[3].bottom = 0;
    rect[3].height = rect[0].height;
}

// handle UI event
void uiEvent(mjuiState *state)
{
    int i;
    char err[200];

    // call UI 0 if event is directed to it
    if ((state->dragrect == ui0.rectid) ||
        (state->dragrect == 0 && state->mouserect == ui0.rectid) ||
        state->type == mjEVENT_KEY)
    {
        // process UI event
        mjuiItem *it = mjui_event(&ui0, state, &con);

        // file section
        if (it && it->sectionid == SECT_FILE)
        {
            switch (it->itemid)
            {
            case 0: // Save xml
                if (!mj_saveLastXML("mjmodel.xml", m, err, 200))
                    printf("Save XML error: %s", err);
                break;

            case 1: // Save mjb
                mj_saveModel(m, "mjmodel.mjb", NULL, 0);
                break;

            case 2: // Print model
                mj_printModel(m, "MJMODEL.TXT");
                break;

            case 3: // Print data
                mj_printData(m, d, "MJDATA.TXT");
                break;

            case 4: // Quit
                settings.exitrequest = 1;
                break;
            }
        }

        // option section
        else if (it && it->sectionid == SECT_OPTION)
        {
            switch (it->itemid)
            {
            case 0: // Spacing
                ui0.spacing = mjui_themeSpacing(settings.spacing);
                ui1.spacing = mjui_themeSpacing(settings.spacing);
                break;

            case 1: // Color
                ui0.color = mjui_themeColor(settings.color);
                ui1.color = mjui_themeColor(settings.color);
                break;

            case 2: // Font
                mjr_changeFont(50 * (settings.font + 1), &con);
                break;

            case 9: // Full screen
                if (glfwGetWindowMonitor(window))
                {
                    // restore window from saved data
                    glfwSetWindowMonitor(window, NULL, windowpos[0], windowpos[1],
                                         windowsize[0], windowsize[1], 0);
                }

                // currently windowed: switch to full screen
                else
                {
                    // save window data
                    glfwGetWindowPos(window, windowpos, windowpos + 1);
                    glfwGetWindowSize(window, windowsize, windowsize + 1);

                    // switch
                    glfwSetWindowMonitor(window, glfwGetPrimaryMonitor(), 0, 0,
                                         vmode.width, vmode.height, vmode.refreshRate);
                }

                // reinstante vsync, just in case
                glfwSwapInterval(settings.vsync);
                break;

            case 10: // Vertical sync
                glfwSwapInterval(settings.vsync);
                break;
            }

            // modify UI
            uiModify(window, &ui0, state, &con);
            uiModify(window, &ui1, state, &con);
        }

        // simulation section
        else if (it && it->sectionid == SECT_SIMULATION)
        {
            switch (it->itemid)
            {
            case 1: // Reset
                if (m)
                {
                    mj_resetData(m, d);
#ifdef SET_INIT_QPOS
                    memcpy(d->qpos, q_init, m->nq * sizeof(mjtNum));
#endif
                    mj_forward(m, d);
                    profilerupdate();
                    sensorupdate();
                    updatesettings();
                }
                break;

            case 2: // Reload
                settings.loadrequest = 1;
                break;

            case 3: // Align
                alignscale();
                updatesettings();
                break;

            case 4: // Copy pose
                copykey();
                break;

            case 5: // Adjust key
            case 6: // Reset to key
                i = settings.key;
                d->time = m->key_time[i];
                mju_copy(d->qpos, m->key_qpos + i * m->nq, m->nq);
                mju_copy(d->qvel, m->key_qvel + i * m->nv, m->nv);
                mju_copy(d->act, m->key_act + i * m->na, m->na);
                mju_copy(d->mocap_pos, m->key_mpos + i * 3 * m->nmocap, 3 * m->nmocap);
                mju_copy(d->mocap_quat, m->key_mquat + i * 4 * m->nmocap, 4 * m->nmocap);
                mj_forward(m, d);
                profilerupdate();
                sensorupdate();
                updatesettings();
                break;

            case 7: // Set key
                i = settings.key;
                m->key_time[i] = d->time;
                mju_copy(m->key_qpos + i * m->nq, d->qpos, m->nq);
                mju_copy(m->key_qvel + i * m->nv, d->qvel, m->nv);
                mju_copy(m->key_act + i * m->na, d->act, m->na);
                mju_copy(m->key_mpos + i * 3 * m->nmocap, d->mocap_pos, 3 * m->nmocap);
                mju_copy(m->key_mquat + i * 4 * m->nmocap, d->mocap_quat, 4 * m->nmocap);
                break;
            }
        }

        // physics section
        else if (it && it->sectionid == SECT_PHYSICS)
        {
            // update disable flags in mjOption
            m->opt.disableflags = 0;
            for (i = 0; i < mjNDISABLE; i++)
                if (settings.disable[i])
                    m->opt.disableflags |= (1 << i);

            // update enable flags in mjOption
            m->opt.enableflags = 0;
            for (i = 0; i < mjNENABLE; i++)
                if (settings.enable[i])
                    m->opt.enableflags |= (1 << i);
        }

        // rendering section
        else if (it && it->sectionid == SECT_RENDERING)
        {
            // set camera in mjvCamera
            if (settings.camera == 0)
                cam.type = mjCAMERA_FREE;
            else if (settings.camera == 1)
            {
                if (pert.select > 0)
                {
                    cam.type = mjCAMERA_TRACKING;
                    cam.trackbodyid = pert.select;
                    cam.fixedcamid = -1;
                }
                else
                {
                    cam.type = mjCAMERA_TRACKING;
                    settings.camera = 1;
                    mjui_update(SECT_RENDERING, -1, &ui0, &uistate, &con);
                }
            }
            else
            {
                cam.type = mjCAMERA_TRACKING;
                cam.fixedcamid = -1;
            }
        }

        // group section
        else if (it && it->sectionid == SECT_GROUP)
        {
            // remake joint section if joint group changed
            if (it->name[0] == 'J' && it->name[1] == 'o')
            {
                ui1.nsect = SECT_JOINT;
                makejoint(ui1.sect[SECT_JOINT].state);
                ui1.nsect = NSECT1;
                uiModify(window, &ui1, state, &con);
            }

            // remake control section if actuator group changed
            if (it->name[0] == 'A' && it->name[1] == 'c')
            {
                ui1.nsect = SECT_CONTROL;
                makecontrol(ui1.sect[SECT_CONTROL].state);
                ui1.nsect = NSECT1;
                uiModify(window, &ui1, state, &con);
            }
        }

        // stop if UI processed event
        if (it != NULL || (state->type == mjEVENT_KEY && state->key == 0))
            return;
    }

    // call UI 1 if event is directed to it
    if ((state->dragrect == ui1.rectid) ||
        (state->dragrect == 0 && state->mouserect == ui1.rectid) ||
        state->type == mjEVENT_KEY)
    {
        // process UI event
        mjuiItem *it = mjui_event(&ui1, state, &con);

        // control section
        if (it && it->sectionid == SECT_CONTROL)
        {
            // clear controls
            if (it->itemid == 0)
            {
                mju_zero(d->ctrl, m->nu);
                mjui_update(SECT_CONTROL, -1, &ui1, &uistate, &con);
            }
        }

        // stop if UI processed event
        if (it != NULL || (state->type == mjEVENT_KEY && state->key == 0))
            return;
    }

    // shortcut not handled by UI
    if (state->type == mjEVENT_KEY && state->key != 0)
    {
        switch (state->key)
        {
        case ' ': // Mode
            if (m)
            {
                settings.run = 1 - settings.run;
                pert.active = 0;
                mjui_update(-1, -1, &ui0, state, &con);
            }
            break;

        case mjKEY_RIGHT: // step forward
            if (m && !settings.run)
            {
                cleartimers();
                mj_step(m, d);
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_LEFT: // step back
            if (m && !settings.run)
            {
                m->opt.timestep = -m->opt.timestep;
                cleartimers();
                mj_step(m, d);
                m->opt.timestep = -m->opt.timestep;
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_DOWN: // step forward 100
            if (m && !settings.run)
            {
                cleartimers();
                for (i = 0; i < 100; i++)
                    mj_step(m, d);
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_UP: // step back 100
            if (m && !settings.run)
            {
                m->opt.timestep = -m->opt.timestep;
                cleartimers();
                for (i = 0; i < 100; i++)
                    mj_step(m, d);
                m->opt.timestep = -m->opt.timestep;
                profilerupdate();
                sensorupdate();
                updatesettings();
            }
            break;

        case mjKEY_PAGE_UP: // select parent body
            if (m && pert.select > 0)
            {
                pert.select = m->body_parentid[pert.select];
                pert.skinselect = -1;

                // stop perturbation if world reached
                if (pert.select <= 0)
                    pert.active = 0;
            }

            break;

        case mjKEY_ESCAPE: // free camera
            cam.type = mjCAMERA_FREE;
            settings.camera = 0;
            mjui_update(SECT_RENDERING, -1, &ui0, &uistate, &con);
            break;
        }

        return;
    }

    // 3D scroll
    if (state->type == mjEVENT_SCROLL && state->mouserect == 3 && m)
    {
        // emulate vertical mouse motion = 5% of window height
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * state->sy, &scn, &cam);

        return;
    }

    // 3D press
    if (state->type == mjEVENT_PRESS && state->mouserect == 3 && m)
    {
        // set perturbation
        int newperturb = 0;
        if (state->control && pert.select > 0)
        {
            // right: translate;  left: rotate
            if (state->right)
                newperturb = mjPERT_TRANSLATE;
            else if (state->left)
                newperturb = mjPERT_ROTATE;

            // perturbation onset: reset reference
            if (newperturb && !pert.active)
                mjv_initPerturb(m, d, &scn, &pert);
        }
        pert.active = newperturb;

        // handle double-click
        if (state->doubleclick)
        {
            // determine selection mode
            int selmode;
            if (state->button == mjBUTTON_LEFT)
                selmode = 1;
            else if (state->control)
                selmode = 3;
            else
                selmode = 2;

            // find geom and 3D click point, get corresponding body
            mjrRect r = state->rect[3];
            mjtNum selpnt[3];
            int selgeom, selskin;
            int selbody = mjv_select(m, d, &vopt,
                                     (mjtNum)r.width / (mjtNum)r.height,
                                     (mjtNum)(state->x - r.left) / (mjtNum)r.width,
                                     (mjtNum)(state->y - r.bottom) / (mjtNum)r.height,
                                     &scn, selpnt, &selgeom, &selskin);

            // set lookat point, start tracking is requested
            if (selmode == 2 || selmode == 3)
            {
                // copy selpnt if anything clicked
                if (selbody >= 0)
                    mju_copy3(cam.lookat, selpnt);

                // switch to tracking camera if dynamic body clicked
                if (selmode == 3 && selbody > 0)
                {
                    // mujoco camera
                    cam.type = mjCAMERA_TRACKING;
                    cam.trackbodyid = selbody;
                    cam.fixedcamid = -1;

                    // UI camera
                    settings.camera = 1;
                    mjui_update(SECT_RENDERING, -1, &ui0, &uistate, &con);
                }
            }

            // set body selection
            else
            {
                if (selbody >= 0)
                {
                    // record selection
                    pert.select = selbody;
                    pert.skinselect = selskin;

                    // compute localpos
                    mjtNum tmp[3];
                    mju_sub3(tmp, selpnt, d->xpos + 3 * pert.select);
                    mju_mulMatTVec(pert.localpos, d->xmat + 9 * pert.select, tmp, 3, 3);
                }
                else
                {
                    pert.select = 0;
                    pert.skinselect = -1;
                }
            }

            // stop perturbation on select
            pert.active = 0;
        }

        return;
    }

    // 3D release
    if (state->type == mjEVENT_RELEASE && state->dragrect == 3 && m)
    {
        // stop perturbation
        pert.active = 0;

        return;
    }

    // 3D move
    if (state->type == mjEVENT_MOVE && state->dragrect == 3 && m)
    {
        // determine action based on mouse button
        mjtMouse action;
        if (state->right)
            action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else if (state->left)
            action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else
            action = mjMOUSE_ZOOM;

        // move perturb or camera
        mjrRect r = state->rect[3];
        if (pert.active)
            mjv_movePerturb(m, d, action, state->dx / r.height, -state->dy / r.height,
                            &scn, &pert);
        else
            mjv_moveCamera(m, action, state->dx / r.height, -state->dy / r.height,
                           &scn, &cam);

        return;
    }
}

//--------------------------- rendering and simulation ----------------------------------

// sim thread synchronization
std::mutex mtx;

// define new geoms
std::array<mjvGeom, 4> planned_footholds;
std::array<mjvGeom, 4> planned_footholds_MRH;

std::deque<mjvGeom> CoM_traj;

std::array<std::array<mjvGeom, 50>, 4> planned_foot_traj;
std::array<std::deque<mjvGeom>, 4> foot_traj;

std::vector<std::vector<mjvGeom>> bound_plane;
std::vector<mjvGeom> center_plane;
std::array<int, MAX_PLANE_NUM> plane_table;

void update_bound_plane()
{
    bound_plane.clear();
    center_plane.clear();
    mjvGeom *g = new mjvGeom;
    mjvGeom *gc = new mjvGeom;
    mjtNum size_line[3] = {1, 1, 1};
    mjtNum size_sphere[3] = {0.01, 0.01, 0.01};
    float rgba_black[4] = {0.2, 0.2, 0.2, 1};
    int plane_id = 0;
    int bound_plane_id = 0;
    for (auto one_plane : perception_planes)
    {
        if (one_plane.bound_num > 0)
        {
            std::vector<mjvGeom> one_bound_plane;
            int bm = one_plane.bound_num;
            for (int i = 0; i < bm; i++)
            {
                Vec3 vertex0;
                vertex0.head(2) = one_plane.Vertexs[i].head(2);
                vertex0[2] = one_plane.A.head(2).dot(one_plane.Vertexs[i]) + one_plane.A[2];
                Vec3 vertex1;
                vertex1.head(2) = one_plane.Vertexs[(i + 1) % bm].head(2);
                vertex1[2] = one_plane.A.head(2).dot(one_plane.Vertexs[(i + 1) % bm]) + one_plane.A[2];
                size_line[2] = (vertex1 - vertex0).norm();
                Mat3 bound_rotation(Eigen::Quaterniond().setFromTwoVectors(vertex1 - vertex0, Vec3(0, 0, 1)));
                mjv_initGeom(g, mjGEOM_LINE, size_line, vertex0.data(), bound_rotation.data(), rgba_black);
                one_bound_plane.push_back(*g);
            }
            bound_plane.push_back(one_bound_plane);
            plane_table[plane_id] = bound_plane_id;
            bound_plane_id++;

            // visualize the center of plane before convex partition
            size_sphere[2] = 0.03;
            Mat3 rot = Mat3::Identity();
            mjv_initGeom(gc, mjGEOM_SPHERE, size_sphere, one_plane.center.data(), rot.data(), rgba_black);
            center_plane.push_back(*gc);
        }
        plane_id++;
    }
    delete g;
    delete gc;
    if (plane_received)
    {
        for (int i = 0; i < 4; i++)
        {
            for (auto id : received_debuginfo.considered_plane[i].id)
            {
                if (id == -1)
                    continue;
                // std::cout << id << std::endl;
                for (int n = 0; n < bound_plane[plane_table[id]].size(); n++)
                {
                    // blue
                    bound_plane[plane_table[id]][n].rgba[2] = 1;
                }
            }
        }
        for (auto id : received_debuginfo.selected_plane_id)
        {
            if (id == -1)
                continue;
            for (int n = 0; n < bound_plane[plane_table[id]].size(); n++)
            {
                // red
                bound_plane[plane_table[id]][n].rgba[0] = 1;
                bound_plane[plane_table[id]][n].rgba[2] = 0.2;
            }
        }
    }
}

// prepare to render
void prepare(void)
{
    // data for FPS calculation
    static double lastupdatetm = 0;

    // update interval, save update time
    double tmnow = glfwGetTime();
    double interval = tmnow - lastupdatetm;
    interval = mjMIN(1, mjMAX(0.0001, interval));
    lastupdatetm = tmnow;

    // no model: nothing to do
    if (!m)
        return;

    // updateHfield();
    update_bound_plane();

    // update scene
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // perpareNewGeoms();
    for (int i = 0; i < CoM_traj.size(); i++)
    {
        mjvGeom *g = scn.geoms + scn.ngeom;
        *g = CoM_traj[i];
        scn.ngeom++;
    }
    for (int n = 0; n < 4; n++)
    {
        for (int i = 0; i < foot_traj[n].size(); i++)
        {
            mjvGeom *g = scn.geoms + scn.ngeom;
            *g = foot_traj[n][i];
            scn.ngeom++;
        }
    }
    for (int i = 0; i < 4; i++)
    {
        mjvGeom *g = scn.geoms + scn.ngeom;
        *g = planned_footholds[i];
        scn.ngeom++;
    }
    for (int i = 0; i < 4; i++)
    {
        mjvGeom *g = scn.geoms + scn.ngeom;
        *g = planned_footholds_MRH[i];
        scn.ngeom++;
    }
    for (int n = 0; n < 4; n++)
    {
        for (int i = 0; i < 50; i++)
        {
            mjvGeom *g = scn.geoms + scn.ngeom;
            *g = planned_foot_traj[n][i];
            scn.ngeom++;
        }
    }
    for (auto one_bound_plane : bound_plane)
    {
        for (auto one_bound : one_bound_plane)
        {
            mjvGeom *g = scn.geoms + scn.ngeom;
            *g = one_bound;
            scn.ngeom++;
        }
    }

    for (auto one_bound_plane : bound_plane)
    {
        for (auto one_bound : one_bound_plane)
        {
            mjvGeom *g = scn.geoms + scn.ngeom;
            *g = one_bound;
            scn.ngeom++;
        }
    }

    for (auto center : center_plane)
    {
        mjvGeom *g = scn.geoms + scn.ngeom;
        *g = center;
        scn.ngeom++;
    }

    // update watch
    if (settings.ui0 && ui0.sect[SECT_WATCH].state)
    {
        watch();
        mjui_update(SECT_WATCH, -1, &ui0, &uistate, &con);
    }

    // ipdate joint
    if (settings.ui1 && ui1.sect[SECT_JOINT].state)
        mjui_update(SECT_JOINT, -1, &ui1, &uistate, &con);

    // update info text
    if (settings.info)
        infotext(info_title, info_content, interval);

    // update profiler
    if (settings.profiler && settings.run)
        profilerupdate();

    // update sensor
    if (settings.sensor && settings.run)
        sensorupdate();

    // clear timers once profiler info has been copied
    cleartimers();
}

// render im main thread (while simulating in background thread)
void render(GLFWwindow *window)
{
    // get 3D rectangle and reduced for profiler
    mjrRect rect = uistate.rect[3];
    mjrRect smallrect = rect;
    if (settings.profiler)
        smallrect.width = rect.width - rect.width / 4;

    // no model
    if (!m)
    {
        // blank screen
        mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);

        // label
        if (settings.loadrequest)
            mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect,
                        "loading", NULL, &con);
        else
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect,
                        "Drag-and-drop model file here", 0, &con);

        // render uis
        if (settings.ui0)
            mjui_render(&ui0, &uistate, &con);
        if (settings.ui1)
            mjui_render(&ui1, &uistate, &con);

        // finalize
        glfwSwapBuffers(window);

        return;
    }

    // render scene
    mjr_render(rect, &scn, &con);

    // show pause/loading label
    if (!settings.run || settings.loadrequest)
        mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect,
                    settings.loadrequest ? "loading" : "pause", NULL, &con);

    // show ui 0
    if (settings.ui0)
        mjui_render(&ui0, &uistate, &con);

    // show ui 1
    if (settings.ui1)
        mjui_render(&ui1, &uistate, &con);

    // show help
    if (settings.help)
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title, help_content, &con);

    // show info
    if (settings.info)
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect,
                    info_title, info_content, &con);

    // show profiler
    if (settings.profiler)
        profilershow(rect);

    // show sensor
    if (settings.sensor)
        sensorshow(smallrect);

    // finalize
    glfwSwapBuffers(window);
}

// simulate in background thread (while rendering in main thread)
void simulate(void)
{
    // cpu-sim syncronization point
    double cpusync = 0;
    mjtNum simsync = 0;

    // run until asked to exit
    while (!settings.exitrequest)
    {
        // sleep for 1 ms or yield, to let main thread run
        //  yield results in busy wait - which has better timing but kills battery life
        if (settings.run && settings.busywait)
            std::this_thread::yield();
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // start exclusive access
        mtx.lock();

        // run only if model is present
        if (m)
        {
            // record start time
            double startwalltm = glfwGetTime();

            // running
            if (settings.run)
            {
                // record cpu time at start of iteration
                double tmstart = glfwGetTime();

                // out-of-sync (for any reason)
                if (d->time < simsync || tmstart < cpusync || cpusync == 0 ||
                    mju_abs((d->time - simsync) - (tmstart - cpusync)) > syncmisalign)
                {
                    // re-sync
                    cpusync = tmstart;
                    simsync = d->time;

                    // clear old perturbations, apply new
                    mju_zero(d->xfrc_applied, 6 * m->nbody);
                    mjv_applyPerturbPose(m, d, &pert, 0); // move mocap bodies only
                    mjv_applyPerturbForce(m, d, &pert);
                }

                // in-sync
                else
                {
                    // step while simtime lags behind cputime, and within safefactor
                    while ((d->time - simsync) < (glfwGetTime() - cpusync) &&
                           (glfwGetTime() - tmstart) < refreshfactor / vmode.refreshRate)
                    {
                        // clear old perturbations, apply new
                        mju_zero(d->xfrc_applied, 6 * m->nbody);
                        mjv_applyPerturbPose(m, d, &pert, 0); // move mocap bodies only
                        mjv_applyPerturbForce(m, d, &pert);

                        // run mj_step
                        mjtNum prevtm = d->time;
                        // break on reset
                        if (d->time < prevtm)
                            break;
                    }
                }
            }

            // paused
            else
            {
                // apply pose perturbation
                mjv_applyPerturbPose(m, d, &pert, 1); // move mocap and dynamic bodies

                // run mj_forward, to update rendering and joint sliders
                mj_forward(m, d);
            }
        }

        // end exclusive access
        mtx.unlock();
    }
}

//-------------------------------- init and main ----------------------------------------

// initalize everything
void init(void)
{
    // print version, check compatibility
    printf("MuJoCo Pro version %.2lf\n", 0.01 * mj_version());
    if (mjVERSION_HEADER != mj_version())
        mju_error("Headers and library have different versions");

    // init GLFW, set timer callback (milliseconds)
    if (!glfwInit())
        mju_error("could not initialize GLFW");
    mjcb_time = timer;

    // multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_VISIBLE, 1);

    // get videomode and save
    vmode = *glfwGetVideoMode(glfwGetPrimaryMonitor());

    // create window
    window = glfwCreateWindow((2 * vmode.width) / 3, (2 * vmode.height) / 3,
                              "State Visualizer", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        mju_error("could not create window");
    }

    // save window position and size
    glfwGetWindowPos(window, windowpos, windowpos + 1);
    glfwGetWindowSize(window, windowsize, windowsize + 1);

    // make context current, set v-sync
    glfwMakeContextCurrent(window);
    glfwSwapInterval(settings.vsync);

    // init abstract visualization
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&vopt);
    profilerinit();
    sensorinit();

    // make empty scene
    mjv_defaultScene(&scn);
    mjv_makeScene(NULL, &scn, maxgeom);

    // select default font
    int fontscale = uiFontScale(window);
    settings.font = fontscale / 50 - 1;
    settings.font = 1;

    // make empty context
    mjr_defaultContext(&con);
    mjr_makeContext(NULL, &con, fontscale);

    // set GLFW callbacks
    uiSetCallback(window, &uistate, uiEvent, uiLayout);
    glfwSetWindowRefreshCallback(window, render);
    glfwSetDropCallback(window, drop);

    // init state and uis
    memset(&uistate, 0, sizeof(mjuiState));
    memset(&ui0, 0, sizeof(mjUI));
    memset(&ui1, 0, sizeof(mjUI));
    ui0.spacing = mjui_themeSpacing(settings.spacing);
    ui0.color = mjui_themeColor(settings.color);
    ui0.predicate = uiPredicate;
    ui0.rectid = 1;
    ui0.auxid = 0;
    ui1.spacing = mjui_themeSpacing(settings.spacing);
    ui1.color = mjui_themeColor(settings.color);
    ui1.predicate = uiPredicate;
    ui1.rectid = 2;
    ui1.auxid = 1;

    // populate uis with standard sections
    mjui_add(&ui0, defFile);
    mjui_add(&ui0, defOption);
    mjui_add(&ui0, defSimulation);
    mjui_add(&ui0, defWatch);
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);
}

void subscribe()
{
    while (lcm_rec.handleTimeout(1) >= 0 && !settings.exitrequest)
        ;
    printf("shutdown lcm subscriber\n");
}

class Handler
{
public:
    ~Handler()
    {
        delete g;
    }

    size_t _iter = 0;
    size_t _iter2 = 0;
    bool inited = false;
    bool inited2 = false;

    double radius_x, radius_y, pos_x, pos_y, max_z;
    int ncols, nrows;

    mjtNum pos[3];
    mjtNum size_traj[3] = {0.005, 0.005, 0.005};
    float rgba_gray[4] = {0.3, 0.3, 0.3, 1};
    float rgba_red[4] = {1, 0, 0, 1};
    mjvGeom *g = new mjvGeom;
    int id[4];
    bool top_arrived = false;
    double final_plane_A3 = 0;
    double final_plane_x = 0;

    void handleMessage(const lcm::ReceiveBuffer *rbuf,
                       const std::string &chan,
                       const StateVis *msg)
    {
        if (_iter % 5 == 0)
        {
            if (m != nullptr)
            {
                if (!inited)
                {
                    id[0] = mj_name2id(m, mjOBJ_GEOM, "fl_foot");
                    id[1] = mj_name2id(m, mjOBJ_GEOM, "fr_foot");
                    id[2] = mj_name2id(m, mjOBJ_GEOM, "rl_foot");
                    id[3] = mj_name2id(m, mjOBJ_GEOM, "rr_foot");
                    inited = true;
                }
                mtx.lock();
                mju_copy(d->qpos, msg->qpos, 19 * sizeof(double));
                mju_copy3(vbody_real.data(), msg->vbody);
                mj_forward(m, d);

                memcpy(pos, d->qpos, 3 * sizeof(mjtNum));
                mjv_initGeom(g, mjGEOM_ELLIPSOID, size_traj, pos, NULL, rgba_gray);
                if (CoM_traj.size() == 0)
                {
                    CoM_traj.push_back(*g);
                }
                else
                {
                    if (float(pos[0]) != CoM_traj.back().pos[0] || float(pos[1]) != CoM_traj.back().pos[1] || float(pos[2]) != CoM_traj.back().pos[2])
                    {
                        CoM_traj.push_back(*g);
                        if (CoM_traj.size() > 150)
                        {
                            CoM_traj.pop_front();
                        }
                    }
                }

                for (int i = 0; i < 4; i++)
                {
                    memcpy(pos, d->geom_xpos + 3 * id[i], 3 * sizeof(mjtNum));
                    mjv_initGeom(g, mjGEOM_ELLIPSOID, size_traj, pos, NULL, rgba_red);
                    if (foot_traj[i].size() == 0)
                    {
                        foot_traj[i].push_back(*g);
                    }
                    else
                    {
                        if (float(pos[0]) != foot_traj[i].back().pos[0] || float(pos[1]) != foot_traj[i].back().pos[1] || float(pos[2]) != foot_traj[i].back().pos[2])
                        {
                            foot_traj[i].push_back(*g);
                            if (foot_traj[i].size() > 100)
                            {
                                foot_traj[i].pop_front();
                            }
                        }
                    }
                }
                mtx.unlock();
            }
        }
    }

    void handleTerrainMessage(const lcm::ReceiveBuffer *rbuf,
                              const std::string &chan,
                              const PerceptionTerrain_lcm *msg)
    {
        plane_num = msg->plane_num;

        for (int i = 0; i < plane_num; i++)
        {
            memcpy(perception_planes[i].A.data(), msg->planes[i].A, 3 * sizeof(double));
            perception_planes[i].bound_num = msg->planes[i].bound_num;
            /* if (i < 4)
            {
                printf("bound_num: %d\n", perception_planes[i].bound_num);
                std::cout << "A: " << perception_planes[i].A.transpose() << std::endl;
            } */
            memcpy(perception_planes[i].center.data(), msg->planes[i].center, 3 * sizeof(double));
            for (int n = 0; n < perception_planes[i].bound_num; n++)
            {
                memcpy(perception_planes[i].Bs[n].data(), msg->planes[i].Bs[n], 3 * sizeof(double));
                memcpy(perception_planes[i].Vertexs[n].data(), msg->planes[i].Vertexs[n], 2 * sizeof(double));
                // if (i < 4)
                // {
                //     std::cout << "(" << i << "," << n << ")"
                //               << " Bs: " << perception_planes[i].Bs[n].transpose() << std::endl;
                //     std::cout << "(" << i << "," << n << ")"
                //               << " Vertex: " << perception_planes[i].Vertexs[n].transpose() << std::endl;
                // }
            }
        }

        for (int i = plane_num; i < MAX_PLANE_NUM; i++)
        {
            perception_planes[i].bound_num = 0;
        }

        int initial_plane_i = plane_num == MAX_PLANE_NUM ? 29 : plane_num;
        perception_planes[initial_plane_i].bound_num = 4;
        perception_planes[initial_plane_i].A << 0, 0, 0;
        perception_planes[initial_plane_i].Bs[0] = Eigen::Vector3d(0, 1, 1.0);
        perception_planes[initial_plane_i].Bs[1] = Eigen::Vector3d(1, 0, 0.65);
        perception_planes[initial_plane_i].Bs[2] = Eigen::Vector3d(0, -1, 1.0);
        perception_planes[initial_plane_i].Bs[3] = Eigen::Vector3d(-1, 0, 2);
        perception_planes[initial_plane_i].Vertexs[0] = Eigen::Vector2d(0.65, 1);
        perception_planes[initial_plane_i].Vertexs[1] = Eigen::Vector2d(0.65, -1);
        perception_planes[initial_plane_i].Vertexs[2] = Eigen::Vector2d(-2, -1);
        perception_planes[initial_plane_i].Vertexs[3] = Eigen::Vector2d(-2, 1);

        /* for (int i = 0; i < MAX_PLANE_NUM; i++)
        {
            printf("\n----------------x----------------\n");
            for (int n = 0; n < perception_planes[i].bound_num; n++)
            {
                printf("%f, ", perception_planes[i].Vertexs[n][0]);
            }
            printf("\n");
            printf("----------------y----------------\n");
            for (int n = 0; n < perception_planes[i].bound_num; n++)
            {
                printf("%f, ", perception_planes[i].Vertexs[n][1]);
            }
        } */

        if (!top_arrived)
        {
            for (int i = 0; i < msg->plane_num; i++)
            {
                auto &plane_i = perception_planes[i];
                if (plane_i.bound_num == 0)
                    continue;
                double span_x = 0;
                for (int n = 1; n < plane_i.bound_num; n++)
                {
                    span_x = abs(plane_i.Vertexs[n - 1].x() - plane_i.Vertexs[n].x());
                    if (span_x > 0.5)
                    {
                        final_plane_A3 = plane_i.A.z();
                        final_plane_x = std::min(plane_i.Vertexs[n - 1].x(), plane_i.Vertexs[n].x());
                        top_arrived = true;
                        break;
                    }
                }
            }
        }
        if (top_arrived)
        {
            int final_plane_i = initial_plane_i == 29 ? 28 : initial_plane_i + 1;
            perception_planes[final_plane_i].bound_num = 4;
            perception_planes[final_plane_i].A << 0, 0, final_plane_A3;
            perception_planes[final_plane_i].Bs[0] = Eigen::Vector3d(0, 1, 1.0);
            perception_planes[final_plane_i].Bs[1] = Eigen::Vector3d(1, 0, final_plane_x + 1);
            perception_planes[final_plane_i].Bs[2] = Eigen::Vector3d(0, -1, 1.0);
            perception_planes[final_plane_i].Bs[3] = Eigen::Vector3d(-1, 0, -final_plane_x);
            perception_planes[final_plane_i].Vertexs[0] = Eigen::Vector2d(final_plane_x + 1, 1);
            perception_planes[final_plane_i].Vertexs[1] = Eigen::Vector2d(final_plane_x + 1, -1);
            perception_planes[final_plane_i].Vertexs[2] = Eigen::Vector2d(final_plane_x, -1);
            perception_planes[final_plane_i].Vertexs[3] = Eigen::Vector2d(final_plane_x, 1);
        }

        for (int i = 0; i < msg->plane_num + 1; i++)
        {
            auto &plane_i = perception_planes[i];
            if (plane_i.bound_num == 0)
                continue;
            double y_span1 = abs(plane_i.Vertexs[1].y() - plane_i.Vertexs[0].y());
            for (int n = 2; n < plane_i.bound_num; n++)
            {
                if (abs(plane_i.Vertexs[n].y() - plane_i.Vertexs[n - 1].y()) > y_span1)
                {
                    y_span1 = abs(plane_i.Vertexs[n].y() - plane_i.Vertexs[n - 1].y());
                }
            }

            for (int j = i + 1; j < msg->plane_num + 1; j++)
            {
                auto &plane_j = perception_planes[j];

                if (plane_j.bound_num == 0)
                    continue;

                if (abs(plane_i.A.z() - plane_j.A.z()) < 0.05)
                {
                    double y_span2 = abs(plane_j.Vertexs[1].y() - plane_j.Vertexs[0].y());
                    for (int n = 2; n < plane_j.bound_num; n++)
                    {
                        if (abs(plane_j.Vertexs[n].y() - plane_j.Vertexs[n - 1].y()) > y_span2)
                        {
                            y_span2 = abs(plane_j.Vertexs[n].y() - plane_j.Vertexs[n - 1].y());
                        }
                    }
                    if (y_span1 >= y_span2)
                    {
                        plane_j.bound_num = 0;
                    }
                    else
                    {
                        plane_i.bound_num = 0;
                    }
                }
            }
        }

        updateHfield_needed = true;

#ifdef SAVE_PLANE
        for (auto &plane : perception_planes)
        {
            save_plane << plane.A.transpose() << " " << plane.bound_num << " ";
            for (int i = 0; i < plane.bound_num; i++)
            {
                save_plane << plane.Bs[i].transpose() << " ";
            }
            for (int i = 0; i < plane.bound_num; i++)
            {
                save_plane << plane.Vertexs[i].transpose() << " ";
            }
            save_plane << std::endl;
        }
#endif
    }

    void handleLocationMessage(const lcm::ReceiveBuffer *rbuf,
                               const std::string &chan,
                               const PerceptionLocation_lcm *msg)
    {
        Vec3 pos;
        Quat quat;
        memcpy(pos.data(), msg->pos, 3 * sizeof(double));
        memcpy(quat.data(), msg->quat, 4 * sizeof(double));

        Mat4 T_camera2body, T_camera2visual, T_visual2origin;

        T_camera2body.setIdentity();
        double theta = 51.4423 / 180.0 * M_PI;
        T_camera2body.topLeftCorner<3, 3>() << 0, sin(theta), -cos(theta),
            -1, 0, 0,
            0, cos(theta), sin(theta);
        T_camera2body.topRightCorner<3, 1>() << 0.3866, 0.0115, -0.03349;

        T_camera2visual.setIdentity();
        T_camera2visual.topLeftCorner<3, 3>() = quaternionToRotationMatrix(quat).transpose();
        T_camera2visual.topRightCorner<3, 1>() = pos;

        T_visual2origin.setIdentity();
        T_visual2origin.topLeftCorner<3, 3>() << 0, 0, -1,
            -1, 0, 0,
            0, 1, 0;
        T_visual2origin.topRightCorner<3, 1>() << 0.3866, 0.0115, 0.03651;

        Mat4 T_body2origin = T_visual2origin * T_camera2visual * T_camera2body.inverse();

        pos = T_body2origin.topRightCorner<3, 1>();
        quat = rotationMatrixToQuaternion(T_body2origin.topLeftCorner<3, 3>().transpose());
        mtx.lock();
        m->geom_pos[3] = pos.x();
        m->geom_pos[4] = pos.y();
        memcpy(d->qpos, pos.data(), 3 * sizeof(double));
        memcpy(d->qpos + 3, quat.data(), 4 * sizeof(double));
        mj_forward(m, d);
        mtx.unlock();
    }
};

class FootholdPlannerDebugHandler
{
public:
    size_t _iter = 0;
    void handleMessage(const lcm::ReceiveBuffer *rbuf,
                       const std::string &chan,
                       const DebugInfo_lcm *msg)
    {
        if (_iter % 5 == 0)
        {
            mtx.lock();
            mjvGeom *g = new mjvGeom;
            mjtNum pos[3];
            mjtNum size_foot[3] = {0.03, 0.03, 0.01};
            mjtNum size_traj[3] = {0.005, 0.005, 0.005};
            float rgba[4] = {0, 0, 1, 1};
            float rgba_MRH[4] = {1, 0, 0, 1};

            for (int i = 0; i < 4; i++)
            {
                memcpy(pos, msg->pFootholds + 3 * i, sizeof(double) * 3);
                mjv_initGeom(g, mjGEOM_ELLIPSOID, size_foot, pos, NULL, rgba);
                planned_footholds[i] = *g;
            }

            for (int i = 0; i < 4; i++)
            {
                memcpy(pos, msg->pFootholds_MRH + 3 * i, sizeof(double) * 3);
                mjv_initGeom(g, mjGEOM_ELLIPSOID, size_foot, pos, NULL, rgba_MRH);
                planned_footholds_MRH[i] = *g;
            }

            for (int i = 0; i < 4; i++)
            {
                for (int n = 0; n < 50; n++)
                {
                    memcpy(pos, msg->pSwingTraj + 3 * 50 * i + 3 * n, sizeof(double) * 3);
                    mjv_initGeom(g, mjGEOM_ELLIPSOID, size_traj, pos, NULL, rgba);
                    planned_foot_traj[i][n] = *g;
                }
            }
            delete g;
            foothold_planner_state = msg->foothold_planner_state;
            swing_planner_state = msg->swing_planner_state;
            foot_planner_time = msg->foot_planner_time;
            cmd_velocity = msg->cmd_velocity;
            received_debuginfo = *msg;
            plane_received = true;
            memcpy(planned_velocity, msg->planned_velocity, sizeof(double) * 3);
            mtx.unlock();
        }
        _iter++;
    }
};

// run event loop
int main(int argc, const char **argv)
{
#ifdef SAVE_PLANE
    save_plane.open(THIS_COM "log/plane.txt", std::ios::ate | std::ios::out);
    if (!save_plane.is_open())
    {
        throw std::runtime_error("file open failed");
    }
#endif

    // initialize everything
    init();

    // request loadmodel if file given (otherwise drag-and-drop)
    if (argc > 1)
    {
        mju_strncpy(filename, argv[1], 1000);
        settings.loadrequest = 1;
    }
    else
    {
        // mju_strncpy(filename, (std::string(THIS_COM) + "/config/xiaotian_es_nw.xml").c_str(), 1000);
        mju_strncpy(filename, (std::string(THIS_COM) + "/config/aliengo.xml").c_str(), 1000);
        settings.loadrequest = 1;
    }

    if (!lcm_rec.good())
        return 1;

    // start simulation thread
    // std::thread simthread(simulate);
    std::thread lcm_rec_thread(subscribe);
    Handler handlerObject;
    FootholdPlannerDebugHandler footholdHandler;
    lcm_rec.subscribe("estimatedState", &Handler::handleMessage, &handlerObject);
    lcm_rec.subscribe("FOOTHOLD_PLANNER_DUBUGINFO", &FootholdPlannerDebugHandler::handleMessage, &footholdHandler);
    lcm_rec.subscribe("PerceptionTerrain", &Handler::handleTerrainMessage, &handlerObject);
    // lcm_rec.subscribe("PerceptionLocation", &Handler::handleLocationMessage, &handlerObject);

    // event loop
    while (!glfwWindowShouldClose(window) && !settings.exitrequest)
    {
        // start exclusive access (block simulation thread)
        mtx.lock();

        // load model (not on first pass, to show "loading" label)
        if (settings.loadrequest == 1)
            loadmodel();
        else if (settings.loadrequest > 1)
            settings.loadrequest = 1;

        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

        // render while simulation is running
        render(window);
    }

    // stop simulation thread
    settings.exitrequest = 1;
    // simthread.join();
    lcm_rec_thread.join();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

#ifdef SAVE_PLANE
    save_plane.close();
#endif
    return 0;
}