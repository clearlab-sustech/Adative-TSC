/*!
 * @file FootSwingTrajectory.cpp
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#include "Interpolation.h"
#include "FootSwingTrajectory.h"

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */
void FootSwingTrajectory::computeSwingTrajectoryBezier(double phase, double swingTime) {
  _p = Interpolate::cubicBezier<Vec3>(_p0, _pf, phase);
  _v = Interpolate::cubicBezierFirstDerivative<Vec3>(_p0, _pf, phase) / swingTime;
  _a = Interpolate::cubicBezierSecondDerivative<Vec3>(_p0, _pf, phase) / (swingTime * swingTime);

  double zp, zv, za;

  if (_using_default_height) {
    double max_height = std::max(_p0[2], _pf[2]);
    if(phase < double(0.5)) {
      zp = Interpolate::cubicBezier<double>(_p0[2], max_height + _height, phase * 2);
      zv = Interpolate::cubicBezierFirstDerivative<double>(_p0[2], max_height + _height, phase * 2) * 2 / swingTime;
      za = Interpolate::cubicBezierSecondDerivative<double>(_p0[2], max_height + _height, phase * 2) * 4 / (swingTime * swingTime);
    } else {
      zp = Interpolate::cubicBezier<double>(max_height + _height, _pf[2], phase * 2 - 1);
      zv = Interpolate::cubicBezierFirstDerivative<double>(max_height + _height, _pf[2], phase * 2 - 1) * 2 / swingTime;
      za = Interpolate::cubicBezierSecondDerivative<double>(max_height + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
    }
  } else {
    zp = _path_coefs[0]*pow(_p[0], 3) + _path_coefs[1]*pow(_p[0], 2) + _path_coefs[2]*_p[0] + _path_coefs[3];
    zv = _v[0] * Vec3(3*_path_coefs[0], 2*_path_coefs[1], _path_coefs[2]).transpose()*Vec3(_p[0]*_p[0], _p[0], 1);
    za = (6*_path_coefs[0]*_p[0]+2*_path_coefs[1])*_v[0]*_v[0] \
        + (3*_path_coefs[0]*_p[0]*_p[0]+2*_path_coefs[1]*_p[0]+_path_coefs[2])*_a[0];
  }

  _p[2] = zp;
  _v[2] = zv;
  _a[2] = za;
}

