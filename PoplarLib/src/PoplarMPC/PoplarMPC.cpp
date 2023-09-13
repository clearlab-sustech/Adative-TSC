//
// Created by nimapng on 1/30/22.
//

#include "PoplarMPC/PoplarMPC.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <algorithm>
#include "Timer.h"

using namespace poplar_mpc;

#define BIG_NUMBER 1e10

PoplarMPC::PoplarMPC(size_t horizon, RealNum dt) : _horizon(horizon), _dt(dt), _gravity(-9.81), _mu(0.4),
                                                   _fmax(500), _setDesiredTraj(false),
                                                   _setDesiredDiscreteTraj(false),
                                                   _ext_wrench(horizon)
{

    _mass = 0;
    _inertia.setZero();
    _Qx.setIdentity();
    _Qf.setZero();
    _Qf.diagonal().fill(1e-3);
    _Q.resize(13 * _horizon, 13 * _horizon);
    _R.resize(12 * _horizon, 12 * _horizon);
    _contactTable = MatInt::Ones(4, horizon);
    _desiredDiscreteTraj = Vec::Zero(13 * horizon);
    _desiredDiscreteTraj_bias = Vec::Zero(13 * horizon);
    _vel_des.setZero();

    _At.resize(13, 13);
    _At.setZero();
    _Bt.resize(13, 12);
    _Bt.setZero();

    // set input constraints
    _n_contact = 4 * _horizon;
    _C.setZero(5 * _n_contact / 2 + 12, 3 * _n_contact / 2 + 12);
    _ub.setZero(5 * _n_contact / 2 + 12);
    _lb.setZero(5 * _n_contact / 2 + 12);

    Sx.resize(13 * _horizon, 13);
    Sx.setZero();
    Su.resize(13 * _horizon, 12 * _horizon / 2 + 12);
    Su.setZero();

    _optimalContactForce.resize(12 * _horizon / 2 + 12);
    _optimalContactForce.setZero();
    _xDot.resize(13 * _horizon);
    _x0.setZero();
    _x0.tail(1) << _gravity;
    fill(_ext_wrench.begin(), _ext_wrench.end(), Vec6::Zero());

    DF.setZero(13 * horizon, 13 * horizon);
    DF.diagonal().head(13).setOnes();
    for (int i = 1; i < horizon; i++)
    {
        DF.diagonal().segment(13 * i, 13) = decay_factor * DF.diagonal().segment(13 * i - 13, 13);
    }

    _fh_n.setZero();
}

void PoplarMPC::setCurrentState(ConstVecRef x0)
{
    assert(x0.size() == 12 || x0.size() == 13);
    if (x0.size() == 12)
    {
        _x0 << x0, _gravity;
    }
    else
    {
        _x0 = x0;
    }
}

void PoplarMPC::setExternalWrench(ConstVec6Ref ext_wrench)
{
    fill(_ext_wrench.begin(), _ext_wrench.end(), ext_wrench);
}

void PoplarMPC::setExternalWrench(ConstVec6Ref ext_wrench, size_t k)
{
    assert(k < _horizon);
    _ext_wrench[k] = ext_wrench;
}

void PoplarMPC::setVelocityCmd(Vec6 vel_des)
{
    _vel_des = vel_des;
}

void PoplarMPC::setDesiredFootZPos(RealNum zd)
{
    _zd = zd;
}

void PoplarMPC::setContactTable(ConstMatIntRef &contactTable)
{
    if (contactTable.rows() != 4 || contactTable.cols() != _horizon)
    {
        throw std::runtime_error("[PoplarMPC::setContactTable] contactTable size is wrong");
    }
    _contactTable = contactTable;
}

void PoplarMPC::setWeight(ConstVecRef Qx, ConstVecRef Qf)
{
    assert(Qx.size() == 12 && Qf.size() == 3);
    _Qx << Qx, 0.0;
    _Qf = Qf;
}

void PoplarMPC::setDesiredTrajectory(const SixDimsPose_Trajectory &traj)
{
    _desiredTraj = traj;
    _setDesiredTraj = true;
}

const SixDimsPose_Trajectory &PoplarMPC::getContinuousOptimizedTrajectory()
{
    return _continuousOptimizedTraj;
}

ConstVecRef PoplarMPC::getDiscreteOptimizedTrajectory()
{
    return ConstVecRef(_discreteOptimizedTraj);
}

ConstVecRef PoplarMPC::getOptimalContactForce()
{
    return ConstVecRef(_optimalContactForce);
}

ConstVecRef PoplarMPC::getCurrentDesiredContactForce()
{
    _force_des = _optimalContactForce.head(12);
    return ConstVecRef(_force_des);
}

ConstVecRef PoplarMPC::getCurrentDesiredFootholds()
{
    return ConstVecRef(_optimalContactForce.tail(12));
}

void PoplarMPC::setMassAndInertia(RealNum mass, Mat3Ref inertia)
{
    _mass = mass;
    _inertia = inertia;
}

void PoplarMPC::solve(RealNum t_now)
{
    if (_setDesiredTraj)
    {
        _setDesiredTraj = false;
        for (int i = 0; i < _horizon; i++)
        {
            _desiredDiscreteTraj.segment(i * 13, 13) << _desiredTraj.roll(t_now + RealNum(i) * _dt).data(),
                _desiredTraj.pitch(t_now + RealNum(i) * _dt).data(),
                _desiredTraj.yaw(t_now + RealNum(i) * _dt).data(),
                _desiredTraj.x(t_now + RealNum(i) * _dt).data(),
                _desiredTraj.y(t_now + RealNum(i) * _dt).data(),
                _desiredTraj.z(t_now + RealNum(i) * _dt).data(),
                _desiredTraj.roll(t_now + RealNum(i) * _dt).derivative(),
                _desiredTraj.pitch(t_now + RealNum(i) * _dt).derivative(),
                _desiredTraj.yaw(t_now + RealNum(i) * _dt).derivative(),
                _desiredTraj.x(t_now + RealNum(i) * _dt).derivative(),
                _desiredTraj.y(t_now + RealNum(i) * _dt).derivative(),
                _desiredTraj.z(t_now + RealNum(i) * _dt).derivative(),
                _gravity;
        }
    }
    else
    {
        if (!_setDesiredDiscreteTraj)
        {
            for (int i = 0; i < _horizon; i++)
            {
                _desiredDiscreteTraj.segment(i * 13, 13)
                    << _x0.head(6) + RealNum(i) * _vel_des * _dt,
                    _vel_des, _gravity;
            }
        }
        else
        {
            _setDesiredDiscreteTraj = false;
        }
    }

    // compensate mpc desired trajectory
    //    std::cout << "_desiredDiscreteTraj: " << _desiredDiscreteTraj.transpose() << std::endl;
    Timer tc1;
    computeSxSu();
    _desiredDiscreteTraj -= _desiredDiscreteTraj_bias;

    // set input constraints
    double mu = 1 / _mu;
    Mat Ci(5, 3);
    Ci << mu, 0, 1.,
        -mu, 0, 1.,
        0, mu, 1.,
        0, -mu, 1.,
        0, 0, 1.;

    _ub.fill(BIG_NUMBER);

    for (int i = 0; i < _horizon / 2 + 1; i++)
    {
        if (i < _horizon / 2)
        {
            for (int leg = 0; leg < 4; leg++)
            {
                int n = (4 * i + leg);
                _C.block(n * 5, n * 3, 5, 3) = Ci;
                _ub(5 * n + 4) = _fmax;
            }
        }
        else
        {
            for (int leg = 0; leg < 4; leg++)
            {
                int nr = 10 * _horizon + ((i - _horizon / 2) * 4 + leg) * 3;
                _C.block(nr, (4 * i + leg) * 3, 3, 3).setIdentity();
                if (_sttr(leg) > 0)
                {
                    _ub.segment(nr, 2) =
                        _currentFootholds[leg].head(2) + (_dt * (_horizon / 2) + _sttr(leg)) * (_x0.segment(9, 2)) + _dt * (_horizon / 2) * _swingVel_max * Vec2::Ones();
                    _lb.segment(nr, 2) =
                        _currentFootholds[leg].head(2) + (_dt * (_horizon / 2) + _sttr(leg)) * (_x0.segment(9, 2)) - _dt * (_horizon / 2) * _swingVel_max * Vec2::Ones();
                    _ub(nr + 2) = _zd;
                    _lb(nr + 2) = _zd - 0.005;
                }
                else
                {
                    _ub.segment(nr, 2) = _currentFootholds[leg].head(2) + _swtr(leg) * (_swingVel_max * Vec2::Ones() + _x0.segment(9, 2));
                    _lb.segment(nr, 2) = _currentFootholds[leg].head(2) - _swtr(leg) * (_swingVel_max * Vec2::Ones() - _x0.segment(9, 2));
                    _ub(nr + 2) = _zd;
                    _lb(nr + 2) = _zd - 0.005;
                }
            }
        }
    }
    // printf("zd: %f\n", _zd);
    // std::cout << "\n--------------------_x0---------------------\n"
    //           << _x0.transpose() << std::endl
    //           << "\n--------------------ub---------------------\n"
    //           << _ub.transpose() << std::endl;

    // set weight matrix
    _Q.setZero(13 * _horizon, 13 * _horizon);
    _Q.diagonal() = _Qx.replicate(_horizon, 1);
    _Q = DF * _Q;
    _R.setZero(3 * _n_contact / 2 + 12, 3 * _n_contact / 2 + 12);
    _R.diagonal() = _Qf.replicate(_n_contact / 2 + 4, 1);
    cout << "Timer tc1: " << tc1.getMs() << endl;

    // formulate QP
    Timer tc2;
    Vec _u_star;
    _u_star.setZero(3 * _n_contact / 2 + 12);
    _u_star.tail(12) = _fh_n;
    Mat par = _Q * Su;
    _H.noalias() = _R + Su.transpose() * par;
    _g.noalias() = par.transpose() * (Sx * _x0 - _desiredDiscreteTraj) - _R * _u_star;
    cout << "Timer tc2: " << tc2.getMs() << endl;
    Timer tc;

    /* Mat Cin(_C.rows() * 2, 3 * _n_contact / 2 + 12);
    Cin << -_C, _C;
    Vec cin(_C.rows() * 2);
    cin << -_lb, _ub;
    myQP = QP_SETUP_dense(_H.rows(), _C.rows(), 0, _H.data(), NULL, Cin.data(), _g.data(), cin.data(), NULL,
                          NULL, ROW_MAJOR_ORDERING);
    QP_SOLVE(myQP);

    memcpy(_optimalContactForce.data(), myQP->x, (3 * _n_contact / 2 + 12) * sizeof(double));

    cout << "qp1: " << _optimalContactForce.transpose() << endl; */

#ifdef USE_QPOASES
    solver = new qpOASES::QProblem(_H.rows(), _C.rows());
    qpOASES::int_t nWSR = 1000;
    solver->reset();
    qpOASES::Options opt;
    opt.setToMPC();
    opt.enableEqualities = qpOASES::BT_TRUE;
    opt.printLevel = qpOASES::PL_NONE;
    solver->setOptions(opt);
    solver->init(_H.data(), _g.data(), _C.data(), nullptr, nullptr, _lb.data(), _ub.data(), nWSR);
    _optimalContactForce.resize(3 * _n_contact / 2 + 12);
    if (solver->isSolved())
    {
        solver->getPrimalSolution(_optimalContactForce.data());
    }
    else
    {
        throw std::runtime_error("poplarMPC qp solver failed");
    }
    delete solver;
    cout << "foothold_plan: " << _optimalContactForce.tail(12).transpose() << endl;
    for (int i = 0; i < 4; i++)
    {
        std::cout << _currentFootholds[i].transpose() << std::endl;
    }
#else
    eiquadprog_solver.reset(_H.rows(), 0, 2 * _C.rows());
    Mat Cin(_C.rows() * 2, 3 * _n_contact / 2 + 12);
    Cin << _C, -_C;
    Vec cin(_C.rows() * 2);
    cin << -_lb, _ub;
    _optimalContactForce.resize(3 * _n_contact / 2 + 12);
    auto state = eiquadprog_solver.solve_quadprog(_H, _g, Mat::Zero(0, 3 * _n_contact / 2 + 12), Vec::Zero(0), Cin, cin,
                                                  _optimalContactForce);
    if (state != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL)
    {
        throw runtime_error("solve() qp failed");
    }
    // cout << "qp2: " << _optimalContactForce.transpose() << endl;
#endif
    cout << "mpc solver time cost: " << tc.getMs() << endl;

    _discreteOptimizedTraj = Sx * _x0 + Su * _optimalContactForce;
    for (int i = 0; i < _horizon; i++)
    {
        computeAtBt(i);
        if (i == 0)
        {
            _xDot.segment(i * 13, 13) =
                _At * _x0 + _Bt * _optimalContactForce.head(12);
        }
        else
        {
            if (i < _horizon / 2)
            {
                _xDot.segment(i * 13, 13) =
                    _At * _discreteOptimizedTraj.segment(i * 13 - 13, 13) +
                    _Bt * _optimalContactForce.segment(12 * i, 12);
            }
            else
            {
                _xDot.segment(i * 13, 13) =
                    _At * _discreteOptimizedTraj.segment(i * 13 - 13, 13) +
                    _Bt * _optimalContactForce.tail(12);
            }
        }
    }

    fill(_ext_wrench.begin(), _ext_wrench.end(), Vec6::Zero());
}

void PoplarMPC::solveImediate(ConstVecRef x0)
{
    _x0 = x0;
    // formulate QP
    Timer tc2;
    Mat par = _Q * Su;
    _H.noalias() = _R + Su.transpose() * par;
    _g.noalias() = par.transpose() * (Sx * _x0 - _desiredDiscreteTraj);
    // cout << "Timer tc2: " << tc2.getMs() << endl;
    Timer tc;
#ifdef USE_QPOASES
    solver = new qpOASES::QProblem(3 * _n_contact, 5 * _n_contact + 2 * _horizon);
    qpOASES::int_t nWSR = 1000;
    solver->reset();
    qpOASES::Options opt;
    opt.setToMPC();
    opt.enableEqualities = qpOASES::BT_TRUE;
    opt.printLevel = qpOASES::PL_NONE;
    solver->setOptions(opt);
    solver->init(_H.data(), _g.data(), _C.data(), nullptr, nullptr, _lb.data(), _ub.data(), nWSR);
    _optimalContactForce.resize(3 * _n_contact);
    if (solver->isSolved())
    {
        solver->getPrimalSolution(_optimalContactForce.data());
    }
    else
    {
        throw std::runtime_error("qp solver failed");
    }
    delete solver;
    // cout << "qp1: " << _optimalContactForce.transpose() << endl;
#else
    eiquadprog_solver.reset(3 * _n_contact, 0, 10 * _n_contact);
    Mat Cin(_C.rows() * 2, 3 * _n_contact);
    Cin << _C, -_C;
    Vec cin(_C.rows() * 2);
    cin << -_lb, _ub;
    _optimalContactForce.resize(3 * _n_contact);
    auto state = eiquadprog_solver.solve_quadprog(_H, _g, Mat::Zero(0, 3 * _n_contact), Vec::Zero(0), Cin, cin,
                                                  _optimalContactForce);
    if (state != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL)
    {
        throw runtime_error("solve() qp failed");
    }
    // cout << "qp2: " << _optimalContactForce.transpose() << endl;
#endif
    // cout << "mpc solver time cost: " << tc.getMs() << endl;

    _discreteOptimizedTraj = Sx * _x0 + Su * _optimalContactForce;
    for (int i = 0; i < _horizon; i++)
    {
        computeAtBt(i);
        if (i == 0)
        {
            _xDot.segment(i * 13, 13) =
                _At * _x0 + _Bt * _optimalContactForce.segment(0, _contactTable.col(i).sum() * 3);
        }
        else
        {
            _xDot.segment(i * 13, 13) =
                _At * _discreteOptimizedTraj.segment(i * 13 - 13, 13) +
                _Bt * _optimalContactForce.segment(_contactTable.leftCols(i).sum() * 3,
                                                   _contactTable.col(i).sum() * 3);
        }
    }

    fill(_ext_wrench.begin(), _ext_wrench.end(), Vec6::Zero());
}

void PoplarMPC::computeSxSu()
{
    for (size_t k = 0; k < _horizon; k++)
    {
        computeAtBtAndBiasTraj(k);
        _Ak = (Mat::Identity(13, 13) + _dt * _At);
        _Bk = _dt * _Bt;
        if (k < _horizon / 2)
        {
            if (k == 0)
            {
                Sx.middleRows(k * 13, 13).noalias() = _Ak;
                Su.topLeftCorner(13, 12).noalias() = _Bk;
            }
            else
            {
                Sx.middleRows(k * 13, 13).noalias() = _Ak * Sx.middleRows((k - 1) * 13, 13);
                size_t sc = 12 * k;
                // printf("k = %ld, sc = %ld, nc = %ld\n", k, sc, _n_contact);
                Su.block(k * 13, 0, 13, sc).noalias() =
                    _Ak * Su.block((k - 1) * 13, 0, 13, sc);
                Su.block(k * 13, sc, 13, 12).noalias() = _Bk;
            }
        }
        else
        {
            Sx.middleRows(k * 13, 13).noalias() = _Ak * Sx.middleRows((k - 1) * 13, 13);
            Su.block(k * 13, 0, 13, (_horizon / 2) * 12).noalias() =
                _Ak * Su.block((k - 1) * 13, 0, 13, (_horizon / 2) * 12);
            Su.block(k * 13, (_horizon / 2) * 12, 13, 12).noalias() = _Ak * Su.block((k - 1) * 13, (_horizon / 2) * 12, 13, 12) + _Bk;
        }
    }
}

void PoplarMPC::setFrictionCoefficient(double mu)
{
    _mu = mu;
}

void PoplarMPC::setContactPointPos(vector<Vec3> contactPointPos)
{
    assert(contactPointPos.size() == 4);
    _contactPointPos = contactPointPos;
}

void PoplarMPC::setCurrentFootholds(vector<Vec3> footholds)
{
    assert(contactPointPos.size() == 4);
    _currentFootholds = footholds;
}

void PoplarMPC::setSwingTimeRemain(Vec4 swtr)
{
    _swtr = swtr;
}

void PoplarMPC::setStanceTimeRemain(Vec4 sttr)
{
    _sttr = sttr;
}

void PoplarMPC::setNorminalFootholds(Vec12 fh_n)
{
    _fh_n = fh_n;
}

void PoplarMPC::setMaxForce(double fmax)
{
    _fmax = fmax;
}

void PoplarMPC::computeAtBt(size_t t)
{
    // TODO: compute At, Bt

    double pc = cos(_x0(1));
    double yc = cos(_x0(2));
    double ys = sin(_x0(2));
    double pt = tan(_x0(1));

    T_inv << yc / pc, ys / pc, 0,
        -ys, yc, 0,
        yc * pt, ys * pt, 1;

    R_wb = rpyToRotMat(_x0.head(3)).transpose();

    Iworld = R_wb * _inertia * R_wb.transpose();
    Iw_inv = Iworld.inverse();

    Vec3 Img = _mass * Vec3(0, 0, -_gravity);
    Mat3 Img_skew;
    Img_skew << 0., -Img(2), Img(1),
        Img(2), 0., -Img(0),
        -Img(1), Img(0), 0.;

    //    std::cout << "------------I_inv*mg_skew---------------\n" << (Iw_inv * Img_skew) << std::endl;

    if (t < _horizon / 2)
    {
        _At.block(0, 6, 3, 3) = T_inv;
        _At.block(3, 9, 3, 3).setIdentity();
        _At.block(6, 3, 3, 3).setZero();
        _At(11, 12) = 1.;
    }
    else
    {
        _At.block(0, 6, 3, 3) = T_inv;
        _At.block(3, 9, 3, 3).setIdentity();
        _At.block(6, 3, 3, 3) = Iw_inv * Img_skew;
        _At(11, 12) = 0.;
    }

    if (t < _horizon / 2)
    {
        for (int leg(0); leg < 4; leg++)
        {
            if (_contactTable(leg, t) == 1)
            {
                Vec3 r = _contactPointPos[leg] - _x0.segment(3, 3) - _dt * (double)t * _x0.segment(9, 3);
                Mat3 r_skew;
                r_skew << 0., -r(2), r(1),
                    r(2), 0., -r(0),
                    -r(1), r(0), 0.;
                _Bt.block(6, leg * 3, 3, 3) = Iw_inv * r_skew;
                _Bt.block(9, leg * 3, 3, 3) = 1 / _mass * Mat3::Identity();
            }
            else
            {
                _Bt.block(6, leg * 3, 3, 3).setZero();
                _Bt.block(9, leg * 3, 3, 3).setZero();
            }
        }
    }
    else
    {
        _Bt.middleRows(9, 3).setZero();
        double n_support_contact = (double)_contactTable.col(t).sum();

        for (int leg(0); leg < 4; leg++)
        {
            if (_contactTable(leg, t) == 1)
            {
                _Bt.block(6, leg * 3, 3, 3) = -1.0 / n_support_contact * Iw_inv * Img_skew;
            }
            else
            {
                _Bt.block(6, leg * 3, 3, 3).setZero();
            }
        }
    }
}

void PoplarMPC::computeAtBtAndBiasTraj(size_t t)
{
    // TODO: compute At, Bt

    double pc = cos(_x0(1));
    double yc = cos(_x0(2));
    double ys = sin(_x0(2));
    double pt = tan(_x0(1));

    T_inv << yc / pc, ys / pc, 0,
        -ys, yc, 0,
        yc * pt, ys * pt, 1;

    R_wb = rpyToRotMat(_x0.head(3)).transpose();

    Iworld = R_wb * _inertia * R_wb.transpose();
    Iw_inv = Iworld.inverse();

    Vec3 Img = _mass * Vec3(0, 0, -_gravity);
    Mat3 Img_skew;
    Img_skew << 0., -Img(2), Img(1),
        Img(2), 0., -Img(0),
        -Img(1), Img(0), 0.;

    //    std::cout << "------------I_inv*mg_skew---------------\n" << (Iw_inv * Img_skew) << std::endl;

    if (t < _horizon / 2)
    {
        _At.block(0, 6, 3, 3) = T_inv;
        _At.block(3, 9, 3, 3).setIdentity();
        _At.block(6, 3, 3, 3).setZero();
        _At(11, 12) = 1.;
    }
    else
    {
        _At.block(0, 6, 3, 3) = T_inv;
        _At.block(3, 9, 3, 3).setIdentity();
        _At.block(6, 3, 3, 3) = Iw_inv * Img_skew;
        _At(11, 12) = 0.;
    }

    Vec3 f_ext, tau_ext;
    f_ext = _ext_wrench[t].head(3);
    tau_ext = _ext_wrench[t].tail(3);
    if (t == 0)
    {
        _desiredDiscreteTraj_bias.segment(13 * t + 6, 3) = _dt * Iw_inv * tau_ext;
        _desiredDiscreteTraj_bias.segment(13 * t + 9, 3) = _dt * f_ext / _mass;
        _desiredDiscreteTraj_bias.segment(13 * t + 3, 3) = 0.5 * _dt * _dt * f_ext / _mass;
        _desiredDiscreteTraj_bias.segment(13 * t, 3) = 0.5 * _dt * _dt * T_inv * (Iw_inv * tau_ext);
    }
    else
    {
        size_t _last = t - 1;
        RealNum tt = 0.5 * _dt * _dt;
        _desiredDiscreteTraj_bias.segment(13 * t + 6, 3) =
            _desiredDiscreteTraj_bias.segment(13 * _last + 6, 3) + _dt * Iw_inv * tau_ext;
        _desiredDiscreteTraj_bias.segment(13 * t + 9, 3) =
            _desiredDiscreteTraj_bias.segment(13 * _last + 9, 3) + _dt * f_ext / _mass;
        _desiredDiscreteTraj_bias.segment(13 * t + 3, 3) =
            _desiredDiscreteTraj_bias.segment(13 * _last + 3, 3) + tt * f_ext / _mass;
        _desiredDiscreteTraj_bias.segment(13 * t, 3) =
            _desiredDiscreteTraj_bias.segment(13 * _last, 3) + tt * T_inv * (Iw_inv * tau_ext);
    }

    if (t < _horizon / 2)
    {
        for (int leg(0); leg < 4; leg++)
        {
            if (_contactTable(leg, t) == 1)
            {
                Vec3 r = _contactPointPos[leg] - _x0.segment(3, 3) - _dt * (double)t * _x0.segment(9, 3);
                Mat3 r_skew;
                r_skew << 0., -r(2), r(1),
                    r(2), 0., -r(0),
                    -r(1), r(0), 0.;
                _Bt.block(6, leg * 3, 3, 3) = Iw_inv * r_skew;
                _Bt.block(9, leg * 3, 3, 3) = 1 / _mass * Mat3::Identity();
            }
            else
            {
                _Bt.block(6, leg * 3, 3, 3).setZero();
                _Bt.block(9, leg * 3, 3, 3).setZero();
            }
        }
    }
    else
    {
        _Bt.middleRows(9, 3).setZero();
        double n_support_contact = (double)_contactTable.col(t).sum();

        for (int leg(0); leg < 4; leg++)
        {
            if (_contactTable(leg, t) == 1)
            {
                _Bt.block(6, leg * 3, 3, 3) = -1.0 / n_support_contact * Iw_inv * Img_skew;
            }
            else
            {
                _Bt.block(6, leg * 3, 3, 3).setZero();
            }
        }
    }

    //    std::cout << "\n----------------At---------------\n" << _At << "\n--------------------Bt---------------------\n"
    //              << _Bt << std::endl;
}

Mat3 PoplarMPC::coordinateRotation(CoordinateAxis axis, double theta)
{
    RealNum s = std::sin(theta);
    RealNum c = std::cos(theta);

    Mat3 R;

    if (axis == CoordinateAxis::X)
    {
        R << 1, 0, 0, 0, c, s, 0, -s, c;
    }
    else if (axis == CoordinateAxis::Y)
    {
        R << c, 0, -s, 0, 1, 0, s, 0, c;
    }
    else if (axis == CoordinateAxis::Z)
    {
        R << c, s, 0, -s, c, 0, 0, 0, 1;
    }

    return R;
}

Mat3 PoplarMPC::rpyToRotMat(Vec3Ref v)
{
    static_assert(v.ColsAtCompileTime == 1 && v.RowsAtCompileTime == 3,
                  "must have 3x1 vector");
    Mat3 m = coordinateRotation(CoordinateAxis::X, v[0]) *
             coordinateRotation(CoordinateAxis::Y, v[1]) *
             coordinateRotation(CoordinateAxis::Z, v[2]);
    return m;
}

ConstVecRef PoplarMPC::getXDot()
{

    return _xDot;
}

void PoplarMPC::setDesiredDiscreteTrajectory(ConstVecRef traj)
{
    assert(traj.size() == 13 * _horizon);
    _desiredDiscreteTraj = traj;
    _setDesiredDiscreteTraj = true;
}

void PoplarMPC::updateCapturableConstraints()
{
    Mat C_cc(2 * _horizon, 13 * _horizon);
    C_cc.setZero();
    Vec lb_cc(2 * _horizon);
    Vec ub_cc(2 * _horizon);
    double ps = sin(_x0[1]);
    double pc = cos(_x0[1]);
    double w = 1.0 / sqrt(9.81 * pc / 0.4); // todo: body height

    for (int i = 0; i < _horizon; i++)
    {
        double g_shift = -0.5 * 9.81 * ps * (_dt * i) * (_dt * i); // Todo;
        //        double g_shift = 0; // Todo;
        double pfx_min, pfx_max, pfy_min, pfy_max;
        for (int leg = 0; leg < 4; leg++)
        {
            if (_contactTable(leg, i) > 0)
            {
                pfx_min = _contactPointPos[leg].x();
                pfx_max = _contactPointPos[leg].x();
                pfx_min = _contactPointPos[leg].y();
                pfy_max = _contactPointPos[leg].y();
                break;
            }
        }
        for (int leg = 0; leg < 4; leg++)
        {
            if (_contactTable(leg, i) > 0)
            {
                if (_contactPointPos[leg].x() < pfx_min)
                    pfx_min = _contactPointPos[leg].x();
                if (_contactPointPos[leg].x() > pfx_max)
                    pfx_max = _contactPointPos[leg].x();
                if (_contactPointPos[leg].y() < pfy_min)
                    pfy_min = _contactPointPos[leg].y();
                if (_contactPointPos[leg].y() > pfy_max)
                    pfy_max = _contactPointPos[leg].y();
            }
        }

        lb_cc(i * 2) = pfx_min - g_shift;
        lb_cc(i * 2 + 1) = pfy_min;
        ub_cc(i * 2) = pfx_max - g_shift;
        ub_cc(i * 2 + 1) = pfy_max;

        C_cc(i * 2, 3) = 1;
        C_cc(i * 2, 9) = w * pc;
        C_cc(i * 2, 11) = w * ps;
        C_cc(i * 2 + 1, 4) = 1;
        C_cc(i * 2 + 1, 10) = w;
    }
    /*cout << "-----------------C_cc-----------------\n" << C_cc << endl
         << "-----------------lb_cc-----------------\n" << lb_cc.transpose() << endl
         << "-----------------ub_cc-----------------\n" << ub_cc.transpose() << endl;*/
    /*_C.bottomRows(2 * _horizon) = C_cc * Su;
    _lb.tail(2 * _horizon) = lb_cc - C_cc * Sx * _x0;
    _ub.tail(2 * _horizon) = ub_cc - C_cc * Sx * _x0;*/
}
