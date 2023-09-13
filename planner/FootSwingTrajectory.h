/*!
 * @file FootSwingTrajectory.h
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#ifndef FOOTSWINGTRAJECTORY_H
#define FOOTSWINGTRAJECTORY_H

#include "cppTypes.h"

/*!
 * A foot swing trajectory for a single foot
 */
class FootSwingTrajectory
{
public:
  /*!
   * Construct a new foot swing trajectory with everything set to zero
   */
  FootSwingTrajectory()
  {
    _p0.setZero();
    _pf.setZero();
    _p.setZero();
    _v.setZero();
    _a.setZero();
    _height = 0;
  }

  /*!
   * Set the starting location of the foot
   * @param p0 : the initial foot position
   */
  void setInitialPosition(Vec3 p0)
  {
    _p0 = p0;
  }

  /*!
   * Set the desired final position of the foot
   * @param pf : the final foot posiiton
   */
  void setFinalPosition(Vec3 pf)
  {
    _pf = pf;
  }

  /*!
   * Set the maximum height of the swing
   * @param h : the maximum height of the swing, achieved halfway through the swing
   */
  void setHeight(double h)
  {
    _height = h;
  }

  void computeSwingTrajectoryBezier(double phase, double swingTime);

  /*!
   * Get the foot position at the current point along the swing
   * @return : the foot position
   */
  Vec3 getPosition()
  {
    return _p;
  }

  Vec3 getFinalPosition()
  {
    return _pf;
  }

  /*!
   * Get the foot velocity at the current point along the swing
   * @return : the foot velocity
   */
  Vec3 getVelocity()
  {
    return _v;
  }

  /*!
   * Get the foot acceleration at the current point along the swing
   * @return : the foot acceleration
   */
  Vec3 getAcceleration()
  {
    return _a;
  }

  void setPathCoefs(Vec4 coefficients) 
  {
    _path_coefs = coefficients;
  }

  void usingDefaultHeight()
  {
    _using_default_height = true;
  }

    void notUsingDefaultHeight()
  {
    _using_default_height = false;
  }

private:
  Vec3 _p0, _pf, _p, _v, _a;
  Vec4 _path_coefs;
  double _height;
  bool _using_default_height;
};

#endif //CHEETAH_SOFTWARE_FOOTSWINGTRAJECTORY_H
