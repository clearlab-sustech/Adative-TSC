#include "generation/SwingTrajectory.h"
#include <core/misc/NumericTraits.h>
#include <yaml-cpp/yaml.h>

namespace clear {

SwingTrajectory::SwingTrajectory(
    Node::SharedPtr nodeHandle,
    std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
    std::shared_ptr<ReferenceBuffer> referenceTrajectoriesBuffer)
    : nodeHandle_(nodeHandle), pinocchioInterface_ptr_(pinocchioInterface_ptr),
      refTrajBuffer_(referenceTrajectoriesBuffer) {
  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();
  auto config_ = YAML::LoadFile(config_file_);
  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
  contact_flag_last = {true, true};
}

SwingTrajectory::~SwingTrajectory() {}

void SwingTrajectory::init() {
  auto mode_schedule = refTrajBuffer_->get_mode_schedule();
  const scalar_t t_now = nodeHandle_->now().seconds();
  if (xf_start_.empty()) {
    for (size_t k = 0; k < foot_names.size(); k++) {
      const auto &foot_name = foot_names[k];
      vector3_t pos =
          pinocchioInterface_ptr_->getFramePose(foot_name).translation();
      std::pair<scalar_t, vector3_t> xs;
      xs.first = t_now;
      xs.second = pos;
      xf_start_[foot_name] = std::move(xs);
    }
  }
  if (xf_end_.empty()) {
    for (size_t k = 0; k < foot_names.size(); k++) {
      const auto &foot_name = foot_names[k];
      vector3_t pos =
          pinocchioInterface_ptr_->getFramePose(foot_name).translation();
      std::pair<scalar_t, vector3_t> xe;
      xe.first = t_now + mode_schedule.get()->duration();
      xe.second = pos;
      xf_end_[foot_name] = std::move(xe);
    }
  }
}

void SwingTrajectory::generate() {
  auto mode_schedule = refTrajBuffer_->get_mode_schedule();
  auto footholds = refTrajBuffer_->get_footholds();
  const scalar_t t_now = nodeHandle_->now().seconds();

  init();

  std::map<std::string, std::shared_ptr<CubicSplineTrajectory>> foot_pos_traj;

  auto contact_flag =
      legged_robot::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(0.0));

  
  for (size_t k = 0; k < foot_names.size(); k++) {
    const auto &foot_name = foot_names[k];
    vector3_t pos =
        pinocchioInterface_ptr_->getFramePose(foot_name).translation();
    if (contact_flag_last[k] != contact_flag[k]) {
      std::pair<scalar_t, vector3_t> xs;
      xs.first = t_now - numeric_traits::limitEpsilon<scalar_t>();
      xs.second = pos;
      // xs.second = footholds[foot_name].second;
      xf_start_[foot_name] = std::move(xs);
    }
    if (contact_flag[k]) {
      if (contact_flag_last[k] != contact_flag[k]) {
        std::pair<scalar_t, vector3_t> xe;
        xe.first = t_now + mode_schedule->timeLeftInMode(0.0);
        xe.second = pos;
        xf_end_[foot_name] = std::move(xe);
      }
    } else {
      std::pair<scalar_t, vector3_t> xe;
      xe.first = footholds[foot_name].first;
      xe.second = footholds[foot_name].second;
      xf_end_[foot_name] = std::move(xe);
    }
    if (xf_start_[foot_name].first <
        xf_end_[foot_name].first - mode_schedule.get()->duration()) {
      xf_start_[foot_name].first =
          xf_end_[foot_name].first - mode_schedule.get()->duration();
      xf_start_[foot_name].second = pos;
    }

    std::vector<scalar_t> time;
    std::vector<vector_t> pos_t;
    time.push_back(xf_start_[foot_name].first);
    pos_t.push_back(xf_start_[foot_name].second);
    time.push_back(0.5 *
                   (xf_start_[foot_name].first + xf_end_[foot_name].first));
    vector3_t middle_pos =
        0.5 * (xf_start_[foot_name].second + xf_end_[foot_name].second);
    middle_pos.z() += contact_flag[k] ? 0.0 : 0.1;
    pos_t.push_back(middle_pos);
    time.push_back(xf_end_[foot_name].first);
    pos_t.push_back(xf_end_[foot_name].second);

    /* std::cout << "############# " << foot_name << ": " << pos.transpose()
              << " ##############\n";
    for (size_t i = 0; i < time.size(); i++) {
      std::cout << " t: " << time[i] - time.front()
                << " pos: " << pos_t[i].transpose() << "\n";
    } */

    auto cubicspline_ = std::make_shared<CubicSplineTrajectory>(
        3, CubicSplineInterpolation::SplineType::cspline);
    cubicspline_->set_boundary(
        CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero(),
        CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero());
    cubicspline_->fit(time, pos_t);
    /* std::cout << "############# opt " << foot_name << " ##############\n";
    for (size_t i = 0; i < time.size(); i++) {
      std::cout
          << " t: " << time[i] - time.front() << " pos: "
          << foot_traj_ptr->pos_trajectoryPtr->evaluate(time[i]).transpose()
          << "\n";
    } */
    foot_pos_traj[foot_name] = std::move(cubicspline_);
  }
  contact_flag_last = contact_flag;
  refTrajBuffer_->set_foot_pos_traj(foot_pos_traj);
}

} // namespace clear
