#include "gait/GaitSchedule.h"
#include "core/misc/Numerics.h"

namespace clear {

GaitSchedule::GaitSchedule(std::string config_yaml) : Node("GaitSchedule") {

  auto config_ = YAML::LoadFile(config_yaml);
  gait_list = config_["gait"]["list"].as<std::vector<std::string>>();

  gait_map_.clear();
  for (const auto &gaitName : gait_list) {
    gait_map_.insert({gaitName, loadGait(config_, gaitName)});
  }

  current_gait_.push(gait_list[0]);
  gait_buffer_.push(gait_list[1]);
  transition_time_.push(this->now().seconds());

  check_.push(true);
  in_transition_.push(false);

  std::string topic_prefix = config_["gait"]["topic_prefix"].as<std::string>();
  std::string mode_schedule_topic =
      config_["gait"]["topic_names"]["mode_schedule"].as<std::string>();
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  mode_schedule_publisher_ =
      this->create_publisher<trans::msg::ModeScheduleTrans>(
          topic_prefix + mode_schedule_topic, qos);

  publish_freq = config_["gait"]["frequency"].as<scalar_t>();
}

void GaitSchedule::start() {
  cycle_timer_ = std::make_shared<CycleTimer>(
      this->shared_from_this(), gait_map_[current_gait_.get()]->duration());
  inner_loop_thread_ = std::thread(&GaitSchedule::inner_loop, this);
  run_.push(true);
}

GaitSchedule::~GaitSchedule() {
  check_.push(false);
  check_transition_thread_.join();
  run_.push(false);
  inner_loop_thread_.join();
}

void GaitSchedule::inner_loop() {
  rclcpp::Rate loop_rate(publish_freq);
  while (rclcpp::ok() && run_.get()) {
    auto mode_schedule = this->eval(current_gait_cycle());
    trans::msg::ModeScheduleTrans msg;
    msg.header.stamp = this->now();
    msg.duration = mode_schedule->duration();
    for (auto &phase : mode_schedule->eventPhases()) {
      msg.event_phases.push_back(phase);
    }
    for (auto &mode : mode_schedule->modeSequence()) {
      msg.mode_sequence.push_back(mode);
    }
    mode_schedule_publisher_->publish(msg);
    loop_rate.sleep();
  }
}

std::shared_ptr<ModeSchedule>
GaitSchedule::loadGait(const YAML::Node node, const std::string &gait_name) {
  scalar_t duration;
  std::vector<scalar_t> eventPhases =
      node["gait"][gait_name]["switchingTimes"].as<std::vector<scalar_t>>();

  scalar_t end_phase = eventPhases.back();
  duration = end_phase;
  if (!numerics::almost_eq(end_phase, 1.0) &&
      end_phase > numeric_traits::limitEpsilon<scalar_t>()) {
    for (auto &phase : eventPhases) {
      phase /= end_phase;
    }
  }

  std::vector<size_t> modeSequence;
  std::vector<std::string> modeSequenceString =
      node["gait"][gait_name]["modeSequence"].as<std::vector<std::string>>();

  if (eventPhases.empty() || modeSequenceString.empty()) {
    throw std::runtime_error("[loadModeSequenceTemplate] failed to load : " +
                             gait_name);
  }
  // convert the mode name to mode enum
  modeSequence.reserve(modeSequenceString.size());
  for (const auto &modeName : modeSequenceString) {
    modeSequence.push_back(quadruped::string2ModeNumber(modeName));
  }
  auto gait =
      std::make_shared<ModeSchedule>(duration, eventPhases, modeSequence);
  if (!gait->isValidModeSequence()) {
    throw std::runtime_error(gait_name + " gait is not valid");
  }
  return gait;
}

void GaitSchedule::switch_gait(std::string gait_name) {
  if (std::find(gait_list.begin(), gait_list.end(), gait_name) !=
      gait_list.end()) {
    if (gait_name != current_gait_.get() && !in_transition_.get()) {
      RCLCPP_WARN(this->get_logger(), "gait request: change to %s",
                  gait_name.c_str());
      if (check_transition_thread_.joinable())
        check_transition_thread_.join();

      auto current_gait_name = current_gait_.get();
      scalar_t phase_ = cycle_timer_->get_cycle_time() /
                        gait_map_[current_gait_name]->duration();
      if (current_gait_name == "stance") {
        transition_time_.push(this->now().seconds() + 0.05);
      } else if (gait_name == "stance") {
        transition_time_.push(
            this->now().seconds() +
            gait_map_[current_gait_name]->timeLeftInModeSequence(phase_) +
            gait_map_[current_gait_name]->duration());
      } else {
        transition_time_.push(
            this->now().seconds() +
            gait_map_[current_gait_name]->timeLeftInModeSequence(phase_));
      }
      gait_buffer_.push(gait_name);
      check_transition_thread_ =
          std::thread(&GaitSchedule::check_gait_transition, this);
      RCLCPP_WARN(this->get_logger(), "gait %s will start at time %f",
                  gait_buffer_.get().c_str(), transition_time_.get());
    }
  }
}

size_t GaitSchedule::current_mode() {
  auto current_gait_name = current_gait_.get();
  scalar_t phase_ =
      cycle_timer_->get_cycle_time() / gait_map_[current_gait_name]->duration();
  return gait_map_[current_gait_name]->getModeFromPhase(phase_);
}

scalar_t GaitSchedule::current_gait_cycle() {
  return gait_map_[current_gait_.get()]->duration();
}

std::shared_ptr<ModeSchedule> GaitSchedule::eval(scalar_t time_period) {
  scalar_t duration;
  std::vector<scalar_t> eventPhases;
  std::vector<size_t> modeSequence;

  auto current_gait_name = current_gait_.get();
  duration = time_period;
  auto &gait_cur = gait_map_[current_gait_name];
  const auto &gait_duration = gait_map_[current_gait_name]->duration();
  const auto &gait_seq = gait_map_[current_gait_name]->modeSequence();
  const auto &gait_event_phase = gait_map_[current_gait_name]->eventPhases();

  scalar_t time_c = cycle_timer_->get_cycle_time();
  scalar_t phase_c = time_c / gait_duration;
  int idx = gait_cur->getModeIndexFromPhase(phase_c);

  eventPhases.push_back(0.0);
  for (size_t i = static_cast<size_t>(idx); i < gait_seq.size(); i++) {
    modeSequence.push_back(gait_seq[i]);
    scalar_t phase_i =
        (gait_event_phase[i + 1] * gait_duration - time_c) / time_period;
    if (phase_i < 1.0) {
      eventPhases.push_back(phase_i);
    } else {
      eventPhases.push_back(1.0);
      break;
    }
  }

  scalar_t time_l = time_period - (gait_duration - time_c);
  scalar_t cycle_n = 1.0;
  while (time_l > 0) {
    for (size_t i = 0; i < gait_seq.size(); i++) {
      modeSequence.push_back(gait_seq[i]);
      scalar_t phase_i =
          ((gait_event_phase[i + 1] + cycle_n) * gait_duration - time_c) /
          time_period;
      if (phase_i < 1.0) {
        eventPhases.push_back(phase_i);
      } else {
        eventPhases.push_back(1.0);
        break;
      }
    }
    cycle_n += 1.0;
    time_l -= gait_duration;
  }
  auto modeSchedule =
      std::make_shared<ModeSchedule>(duration, eventPhases, modeSequence);
  if (!modeSchedule->isValidModeSequence()) {
    throw std::runtime_error(
        "GaitSchedule::eval >>> modeSchedule is not valid");
  }
  return modeSchedule;
}

void GaitSchedule::check_gait_transition() {
  in_transition_.push(true);

  if (transition_time_.get() < this->now().seconds() ||
      transition_time_.get() - this->now().seconds() > 1e2) {
    printf("transition time slot: %fs",
           transition_time_.get() - this->now().seconds());
    throw std::runtime_error("transition time slot is not valid [0, 100]");
  }

  rclcpp::Rate loop_rate(1000.0);
  while (rclcpp::ok() && check_.get() &&
         gait_buffer_.get() != current_gait_.get()) {
    if (this->now().seconds() > transition_time_.get()) {
      current_gait_.push(gait_buffer_.get());
      cycle_timer_ = std::make_shared<CycleTimer>(
          this->shared_from_this(), gait_map_[current_gait_.get()]->duration());
    }
    loop_rate.sleep();
  }
  RCLCPP_WARN(this->get_logger(), "gait has been changed to %s",
              current_gait_.get().c_str());

  in_transition_.push(false);
}
std::string GaitSchedule::get_current_gait_name() {
  return current_gait_.get();
}

} // namespace clear
