#include "pinocchio/PinocchioInterface.h"
#include <rcpputils/asserts.hpp>

namespace clear {
PinocchioInterface::PinocchioInterface(const char *urdf_name) {
  pin::urdf::buildModel(std::string(urdf_name), pin::JointModelFreeFlyer(),
                        this->model_);
  this->data_ = pin::Data(this->model_);
  qpos_.setZero(model_.nq);
  qvel_.setZero(model_.nv);
}

PinocchioInterface::~PinocchioInterface() {}

const pin::Model &PinocchioInterface::getModel() { return model_; }

const pin::Data &PinocchioInterface::getData() { return data_; }

void PinocchioInterface::updateRobotState(vector_t qpos, vector_t qvel) {
  qpos_ = std::move(qpos);
  qvel_ = std::move(qvel);

  pin::normalize(model_, qpos_);
  pin::computeAllTerms(model_, data_, qpos_, qvel_);
  pin::computeCentroidalMomentumTimeVariation(model_, data_, qpos_, qvel_,
                                              vector_t::Zero(Nv()));
  pinocchio::ccrba(model_, data_, qpos_, qvel_);
  pin::computeMinverse(model_, data_, qpos_);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::centerOfMass(model_, data_, qpos_, qvel_,
                          vector_t::Zero(model_.nv));
  data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  data_.Minv.triangularView<Eigen::StrictlyLower>() =
      data_.Minv.transpose().triangularView<Eigen::StrictlyLower>();
}

scalar_t PinocchioInterface::total_mass() {
  return pin::computeTotalMass(model_);
}

vector3_t PinocchioInterface::getCoMPos() { return data_.com[0]; }

vector3_t PinocchioInterface::getCoMVel() { return data_.vcom[0]; }

vector3_t PinocchioInterface::getCoMAcc() { return data_.acom[0]; }

const matrix3x_t &PinocchioInterface::getJacobia_CoM() { return data_.Jcom; }

void PinocchioInterface::getJacobia_local(string frame_name, matrix6x_t &J) {
  if (J.size() != 6 * model_.nv) {
    J.resize(6, model_.nv);
  }
  J.setZero();
  pin::getFrameJacobian(model_, data_, getFrameID(frame_name), pin::LOCAL, J);
}

void PinocchioInterface::getJacobia_world(string frame_name, matrix6x_t &J) {
  if (J.size() != 6 * model_.nv) {
    J.resize(6, model_.nv);
  }
  J.setZero();
  pin::getFrameJacobian(model_, data_, getFrameID(frame_name), pin::WORLD, J);
}

void PinocchioInterface::getJacobia_localWorldAligned(string frame_name,
                                                      matrix6x_t &J) {
  if (J.size() != 6 * model_.nv) {
    J.resize(6, model_.nv);
  }
  J.setZero();
  pin::getFrameJacobian(model_, data_, getFrameID(frame_name),
                        pin::LOCAL_WORLD_ALIGNED, J);
}

pin::FrameIndex PinocchioInterface::getFrameID(string frame_name) {
  rcpputils::assert_true(model_.existFrame(frame_name));
  return model_.getFrameId(frame_name);
}

Eigen::Ref<matrix_t> PinocchioInterface::M() {
  return Eigen::Ref<matrix_t>(data_.M);
}

matrix_t PinocchioInterface::Minv() { return data_.Minv; }

Eigen::Ref<vector_t> PinocchioInterface::nle() {
  return Eigen::Ref<vector_t>(data_.nle);
}

const pin::SE3 &PinocchioInterface::getFramePose(string frame_name) {
  return data_.oMf[getFrameID(frame_name)];
}

pin::Motion PinocchioInterface::getFrame6dVel_local(string frame_name) {

  return pin::getFrameVelocity(model_, data_, getFrameID(frame_name),
                               pin::LOCAL);
}

pin::Motion
PinocchioInterface::getFrame6dVel_localWorldAligned(string frame_name) {
  return pin::getFrameVelocity(model_, data_, getFrameID(frame_name),
                               pin::LOCAL_WORLD_ALIGNED);
}

pin::Motion PinocchioInterface::getFrame6dAcc_local(string frame_name) {
  return pin::getFrameAcceleration(model_, data_, getFrameID(frame_name),
                                   pin::LOCAL);
}

pin::Motion PinocchioInterface::getFrame6dAcc_world(string frame_name) {
  return pin::getFrameAcceleration(model_, data_, getFrameID(frame_name),
                                   pin::WORLD);
}

pin::Motion
PinocchioInterface::getFrame6dAcc_localWorldAligned(string frame_name) {
  return pin::getFrameAcceleration(model_, data_, getFrameID(frame_name),
                                   pin::LOCAL_WORLD_ALIGNED);
}

int PinocchioInterface::Nq() { return model_.nq; }

int PinocchioInterface::Nv() { return model_.nv; }

Eigen::Ref<vector_t> PinocchioInterface::qpos() {
  return Eigen::Ref<vector_t>(qpos_);
}

Eigen::Ref<vector_t> PinocchioInterface::qvel() {
  return Eigen::Ref<vector_t>(qvel_);
}

const matrix6x_t &PinocchioInterface::momentumJacobia() { return data_.Ag; }

} // namespace clear