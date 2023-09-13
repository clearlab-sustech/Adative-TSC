//
// Created by nimpng on 7/4/21.
//

#include "SRGB_MPC/SRGB_MPC.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <algorithm>
#include "Timer.h"

using namespace SRGB_MPC;

#define BIG_NUMBER 1e10

SRGB_MPC_IMPL::SRGB_MPC_IMPL(size_t horizon, RealNum dt) : _horizon(horizon), _dt(dt), _gravity(-9.81), _mu(0.4),
                                                           _fmax(500), _setDesiredTraj(false),
                                                           _setDesiredDiscreteTraj(false),
                                                           _ext_wrench(horizon) {

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
    _At.block(3, 9, 3, 3).setIdentity();
    _At(11, 12) = 1.;
    _Bt.resize(13, 12);
    _Bt.setZero();

    Sx.resize(13 * _horizon, 13);
    Sx.setZero();
    Su.resize(13 * _horizon, 12 * _horizon);
    Su.setZero();

    _optimalContactForce.resize(12 * _horizon);
    _optimalContactForce.setZero();
    _xDot.resize(13 * _horizon);
    _x0.setZero();
    _x0.tail(1) << _gravity;
    fill(_ext_wrench.begin(), _ext_wrench.end(), Vec6::Zero());

    DF.setZero(13 * horizon, 13 * horizon);
    DF.diagonal().head(13).setOnes();
    for (int i = 1; i < horizon; i++) {
        DF.diagonal().segment(13 * i, 13) = decay_factor * DF.diagonal().segment(13 * i - 13, 13);
    }
}

void SRGB_MPC_IMPL::setCurrentState(ConstVecRef x0) {
    assert(x0.size() == 12 || x0.size() == 13);
    if (x0.size() == 12) {
        _x0 << x0, _gravity;
    } else {
        _x0 = x0;
    }
}

void SRGB_MPC_IMPL::setExternalWrench(ConstVec6Ref ext_wrench) {
    fill(_ext_wrench.begin(), _ext_wrench.end(), ext_wrench);
}

void SRGB_MPC_IMPL::setExternalWrench(ConstVec6Ref ext_wrench, size_t k) {
    assert(k < _horizon);
    _ext_wrench[k] = ext_wrench;
}

void SRGB_MPC_IMPL::setVelocityCmd(Vec6 vel_des) {
    _vel_des = vel_des;
}

void SRGB_MPC_IMPL::setContactTable(ConstMatIntRef &contactTable) {
    if (contactTable.rows() != 4 || contactTable.cols() != _horizon) {
        throw std::runtime_error("[SRGB_MPC_IMPL::setContactTable] contactTable size is wrong");
    }
    _contactTable = contactTable;
}

void SRGB_MPC_IMPL::setWeight(ConstVecRef Qx, ConstVecRef Qf) {
    assert(Qx.size() == 12 && Qf.size() == 3);
    _Qx << Qx, 0.0;
    _Qf = Qf;
}

void SRGB_MPC_IMPL::setDesiredTrajectory(const SixDimsPose_Trajectory &traj) {
    _desiredTraj = traj;
    _setDesiredTraj = true;
}

const SixDimsPose_Trajectory &SRGB_MPC_IMPL::getContinuousOptimizedTrajectory() {
    return _continuousOptimizedTraj;
}

ConstVecRef SRGB_MPC_IMPL::getDiscreteOptimizedTrajectory() {
    return SRGB_MPC::ConstVecRef(_discreteOptimizedTraj);
}

ConstVecRef SRGB_MPC_IMPL::getOptimalContactForce() {
    return SRGB_MPC::ConstVecRef(_optimalContactForce);
}

ConstVecRef SRGB_MPC_IMPL::getCurrentDesiredContactForce() {
    _force_des.setZero();
    int k = 0;
    for (int i = 0; i < 4; i++) {
        if (_contactTable.col(0)(i) == 1) {
            _force_des.segment(i * 3, 3) = _optimalContactForce.segment(3 * k, 3);
            k++;
        }
    }
    return SRGB_MPC::ConstVecRef(_force_des);
}

void SRGB_MPC_IMPL::setMassAndInertia(RealNum mass, Mat3Ref inertia) {
    _mass = mass;
    _inertia = inertia;
}

void SRGB_MPC_IMPL::solve(RealNum t_now) {

    if (_setDesiredTraj) {
        _setDesiredTraj = false;
        for (int i = 0; i < _horizon; i++) {
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
    } else {
        if (!_setDesiredDiscreteTraj) {
            for (int i = 0; i < _horizon; i++) {
                _desiredDiscreteTraj.segment(i * 13, 13)
                        << _x0.head(6) + RealNum(i) * _vel_des * _dt,
                        _vel_des, _gravity;
            }
        } else {
            _setDesiredDiscreteTraj = false;
        }
    }

    // compensate mpc desired trajectory

    //    std::cout << "_desiredDiscreteTraj: " << _desiredDiscreteTraj.transpose() << std::endl;
    _n_contact = _contactTable.sum();
    Timer tc1;
    computeSxSu();
    _desiredDiscreteTraj -= _desiredDiscreteTraj_bias;

    // set input constraints
    _C.setZero(5 * _n_contact + 2 * _horizon, 3 * _n_contact);
    _ub.setZero(5 * _n_contact + 2 * _horizon);
    _lb.setZero(5 * _n_contact + 2 * _horizon);

    double mu = 1 / _mu;
    Mat Ci(5, 3);
    Ci << mu, 0, 1.,
            -mu, 0, 1.,
            0, mu, 1.,
            0, -mu, 1.,
            0, 0, 1.;

    _ub.fill(BIG_NUMBER);
    for (int i = 0; i < _n_contact; i++) {
        _C.block(i * 5, i * 3, 5, 3) = Ci;
        _ub(5 * i + 4) = _fmax;
    }

    updateCapturableConstraints();

    // set weight matrix
    _Q.setZero(13 * _horizon, 13 * _horizon);
    _Q.diagonal() = _Qx.replicate(_horizon, 1);
    _Q = DF * _Q;
    _R.setZero(3 * _n_contact, 3 * _n_contact);
    _R.diagonal() = _Qf.replicate(_n_contact, 1);
    cout << "Timer tc1: " << tc1.getMs() << endl;

    // formulate QP
    Timer tc2;
    Mat par = _Q * Su;
    _H.noalias() = _R + Su.transpose() * par;
    _g.noalias() = par.transpose() * (Sx * _x0 - _desiredDiscreteTraj);
    cout << "Timer tc2: " << tc2.getMs() << endl;
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
    if (solver->isSolved()) {
        solver->getPrimalSolution(_optimalContactForce.data());
    } else {
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
    auto state = eiquadprog_solver.solve_quadprog(_H, _g, Mat::Zero(0, 3 * _n_contact), Vec::Zero(0), Cin, cin, _optimalContactForce);
    if (state != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL)
    {
        throw runtime_error("SRGB_MPC::solve() qp failed");
    }
    // cout << "qp2: " << _optimalContactForce.transpose() << endl;
#endif
    cout << "mpc solver time cost: " << tc.getMs() << endl;

    _discreteOptimizedTraj = Sx * _x0 + Su * _optimalContactForce;
    for (int i = 0; i < _horizon; i++) {
        computeAtBt(i);
        if (i == 0) {
            _xDot.segment(i * 13, 13) =
                    _At * _x0 + _Bt * _optimalContactForce.segment(0, _contactTable.col(i).sum() * 3);
        } else {
            _xDot.segment(i * 13, 13) =
                    _At * _discreteOptimizedTraj.segment(i * 13 - 13, 13) +
                    _Bt * _optimalContactForce.segment(_contactTable.leftCols(i).sum() * 3,
                                                       _contactTable.col(i).sum() * 3);
        }
    }

    fill(_ext_wrench.begin(), _ext_wrench.end(), Vec6::Zero());
}

void SRGB_MPC_IMPL::solveImediate(ConstVecRef x0) {
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
    if (solver->isSolved()) {
        solver->getPrimalSolution(_optimalContactForce.data());
    } else {
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
    auto state = eiquadprog_solver.solve_quadprog(_H, _g, Mat::Zero(0, 3 * _n_contact), Vec::Zero(0), Cin, cin, _optimalContactForce);
    if (state != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL)
    {
        throw runtime_error("SRGB_MPC::solve() qp failed");
    }
    // cout << "qp2: " << _optimalContactForce.transpose() << endl;
#endif
    // cout << "mpc solver time cost: " << tc.getMs() << endl;

    _discreteOptimizedTraj = Sx * _x0 + Su * _optimalContactForce;
    for (int i = 0; i < _horizon; i++) {
        computeAtBt(i);
        if (i == 0) {
            _xDot.segment(i * 13, 13) =
                    _At * _x0 + _Bt * _optimalContactForce.segment(0, _contactTable.col(i).sum() * 3);
        } else {
            _xDot.segment(i * 13, 13) =
                    _At * _discreteOptimizedTraj.segment(i * 13 - 13, 13) +
                    _Bt * _optimalContactForce.segment(_contactTable.leftCols(i).sum() * 3,
                                                       _contactTable.col(i).sum() * 3);
        }
    }

    fill(_ext_wrench.begin(), _ext_wrench.end(), Vec6::Zero());
}

void SRGB_MPC_IMPL::computeSxSu() {

    Sx.setZero(13 * _horizon, 13);
    Su.setZero(13 * _horizon, 3 * _n_contact);

    for (size_t k = 0; k < _horizon; k++) {
        computeAtBtAndBiasTraj(k);
//        Mat P0(_At.rows() + _Bt.cols(), _At.cols() + _Bt.cols());
//        P0.setZero();
//        P0.topRows(_At.rows()) << _At, _Bt;
//        P0 *= _dt;
//        Mat P0_exp = P0.exp();
//        _Ak = P0_exp.topLeftCorner(_At.rows(), _At.cols());
//        _Bk = P0_exp.topRightCorner(_Bt.rows(), _Bt.cols());
        _Ak = (Mat::Identity(13, 13) + _dt * _At);
        _Bk = _dt * _Bt;
//        std::cout << "----------------" << k << "------------------" << std::endl
//                  << "----------------------Ak-----------------------\n"
//                  << _Ak << std::endl
//                  << "----------------I+At*dt----------------\n"
//                  << (Mat::Identity(13, 13) + _dt * _At) << std::endl;
        if (k == 0) {
            Sx.middleRows(k * 13, 13).noalias() = _Ak;
            Su.topLeftCorner(13, _contactTable.col(k).sum() * 3).noalias() = _Bk;
        } else {
            Sx.middleRows(k * 13, 13).noalias() = _Ak * Sx.middleRows((k - 1) * 13, 13);
            size_t sc = 3 * _contactTable.leftCols(k).sum();
            // printf("k = %ld, sc = %ld, nc = %ld\n", k, sc, _n_contact);
            Su.block(k * 13, 0, 13, sc).noalias() =
                    _Ak * Su.block((k - 1) * 13, 0, 13, sc);
            Su.block(k * 13, sc, 13, _contactTable.col(k).sum() * 3).noalias() = _Bk;
        }
    }
}

void SRGB_MPC_IMPL::setFrictionCoefficient(double mu) {
    _mu = mu;
}

void SRGB_MPC_IMPL::setContactPointPos(vector<Vec3> contactPointPos) {
    assert(contactPointPos.size() == 4);
    _contactPointPos = contactPointPos;
}

void SRGB_MPC_IMPL::setMaxForce(double fmax) {
    _fmax = fmax;
}

void SRGB_MPC_IMPL::computeAtBt(size_t t) {
    // TODO: compute At, Bt

    double pc = cos(_x0(1));
    double yc = cos(_x0(2));
    double ys = sin(_x0(2));
    double pt = tan(_x0(1));

    T_inv << yc / pc, ys / pc, 0,
            -ys, yc, 0,
            yc * pt, ys * pt, 1;

    R_wb = rpyToRotMat(_x0.head(3)).transpose();

    _At.block(0, 6, 3, 3) = T_inv;

    Iworld = R_wb * _inertia * R_wb.transpose();
    Iw_inv = Iworld.inverse();

    size_t n_support_contact = _contactTable.col(t).sum();
    _Bt.setZero(13, 3 * n_support_contact);
    Vec3 r;
    int k = 0;
    for (int leg(0); leg < 4; leg++) {
        if (_contactTable(leg, t) == 1) {
            r = _contactPointPos[leg] - _x0.segment(3, 3) - _dt * (double) t * _desiredDiscreteTraj.segment(9, 3);
            Mat3 r_skew;
            r_skew << 0., -r(2), r(1),
                    r(2), 0., -r(0),
                    -r(1), r(0), 0.;
            _Bt.block(6, k * 3, 3, 3) = Iw_inv * r_skew;
            _Bt.block(9, k * 3, 3, 3) = 1 / _mass * Mat3::Identity();
            k++;
        }
    }
}

void SRGB_MPC_IMPL::computeAtBtAndBiasTraj(size_t t) {
    // TODO: compute At, Bt

    double pc = cos(_x0(1));
    double yc = cos(_x0(2));
    double ys = sin(_x0(2));
    double pt = tan(_x0(1));

    T_inv << yc / pc, ys / pc, 0,
            -ys, yc, 0,
            yc * pt, ys * pt, 1;

    R_wb = rpyToRotMat(_x0.head(3)).transpose();

    _At.block(0, 6, 3, 3) = T_inv;

    Iworld = R_wb * _inertia * R_wb.transpose();
    Iw_inv = Iworld.inverse();

    Vec3 f_ext, tau_ext;
    f_ext = _ext_wrench[t].head(3);
    tau_ext = _ext_wrench[t].tail(3);
    if (t == 0) {
        _desiredDiscreteTraj_bias.segment(13 * t + 6, 3) = _dt * Iw_inv * tau_ext;
        _desiredDiscreteTraj_bias.segment(13 * t + 9, 3) = _dt * f_ext / _mass;
        _desiredDiscreteTraj_bias.segment(13 * t + 3, 3) = 0.5 * _dt * _dt * f_ext / _mass;
        _desiredDiscreteTraj_bias.segment(13 * t, 3) = 0.5 * _dt * _dt * T_inv * (Iw_inv * tau_ext);
    } else {
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

    size_t n_support_contact = _contactTable.col(t).sum();
    _Bt.resize(13, 3 * n_support_contact);
    _Bt.setZero();
    Vec3 r;
    int k = 0;
    for (int leg(0); leg < 4; leg++) {
        if (_contactTable(leg, t) == 1) {
            r = _contactPointPos[leg] - _x0.segment(3, 3) - _dt * (double) t * _x0.segment(9, 3);
            Mat3 r_skew;
            r_skew << 0., -r(2), r(1),
                    r(2), 0., -r(0),
                    -r(1), r(0), 0.;
            _Bt.block(6, k * 3, 3, 3) = Iw_inv * r_skew;
            _Bt.block(9, k * 3, 3, 3) = 1 / _mass * Mat3::Identity();
            k++;
        }
    }
}

Mat3 SRGB_MPC_IMPL::coordinateRotation(CoordinateAxis axis, double theta) {
    RealNum s = std::sin(theta);
    RealNum c = std::cos(theta);

    Mat3 R;

    if (axis == CoordinateAxis::X) {
        R << 1, 0, 0, 0, c, s, 0, -s, c;
    } else if (axis == CoordinateAxis::Y) {
        R << c, 0, -s, 0, 1, 0, s, 0, c;
    } else if (axis == CoordinateAxis::Z) {
        R << c, s, 0, -s, c, 0, 0, 0, 1;
    }

    return R;
}

Mat3 SRGB_MPC_IMPL::rpyToRotMat(Vec3Ref v) {
    static_assert(v.ColsAtCompileTime == 1 && v.RowsAtCompileTime == 3,
                  "must have 3x1 vector");
    Mat3 m = coordinateRotation(CoordinateAxis::X, v[0]) *
             coordinateRotation(CoordinateAxis::Y, v[1]) *
             coordinateRotation(CoordinateAxis::Z, v[2]);
    return m;
}

ConstVecRef SRGB_MPC_IMPL::getXDot() {

    return _xDot;
}

void SRGB_MPC_IMPL::setDesiredDiscreteTrajectory(ConstVecRef traj) {
    assert(traj.size() == 13 * _horizon);
    _desiredDiscreteTraj = traj;
    _setDesiredDiscreteTraj = true;
}

void SRGB_MPC_IMPL::updateCapturableConstraints() {
    Mat C_cc(2 * _horizon, 13 * _horizon);
    C_cc.setZero();
    Vec lb_cc(2 * _horizon);
    Vec ub_cc(2 * _horizon);
    double ps = sin(_x0[1]);
    double pc = cos(_x0[1]);
    double w = 1.0 / sqrt(9.81 * pc / 0.4); // todo: body height

    for (int i = 0; i < _horizon; i++) {
        double g_shift = -0.5 * 9.81 * ps * (_dt * i) * (_dt * i); // Todo;
        //        double g_shift = 0; // Todo;
        double pfx_min, pfx_max, pfy_min, pfy_max;
        for (int leg = 0; leg < 4; leg++) {
            if (_contactTable(leg, i) > 0) {
                pfx_min = _contactPointPos[leg].x();
                pfx_max = _contactPointPos[leg].x();
                pfx_min = _contactPointPos[leg].y();
                pfy_max = _contactPointPos[leg].y();
                break;
            }
        }
        for (int leg = 0; leg < 4; leg++) {
            if (_contactTable(leg, i) > 0) {
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
