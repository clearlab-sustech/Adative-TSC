#include "estimation/StateEstimationLKF.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

<<<<<<< HEAD
namespace clear {

StateEstimationLKF::StateEstimationLKF(Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {

  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);
  log_dir = config_["controller"]["log_dir"].as<std::string>();

  robot_name = config_["model"]["name"].as<std::string>();
  RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "robot_name: %s",
              robot_name.c_str());

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "model file: %s",
              urdf.c_str());

  pinocchioInterface_ptr = std::make_unique<PinocchioInterface>(urdf.c_str());
  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
  dt_ = config_["estimation"]["dt"].as<scalar_t>();
  use_odom_ = config_["estimation"]["use_odom"].as<bool>();

  RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "dt: %f", dt_);
  RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "use odom: %s",
              use_odom_ ? "true" : "false");
  for (const auto &name : foot_names) {
    RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "foot name: %s",
                name.c_str());
    cflag_.emplace_back(true);
  }

  bool hardware_ = config_["estimation"]["hardware"].as<bool>();

  if (!hardware_) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    std::string topic_prefix =
        config_["global"]["topic_prefix"].as<std::string>();
    std::string imu_topic =
        config_["global"]["topic_names"]["imu"].as<std::string>();
    imu_subscription_ = nodeHandle_->create_subscription<sensor_msgs::msg::Imu>(
        topic_prefix + imu_topic, qos,
        std::bind(&StateEstimationLKF::imuCallback, this,
                  std::placeholders::_1));

    std::string touch_sensor_topic =
        config_["global"]["topic_names"]["touch_sensor"].as<std::string>();
    touch_subscription_ =
        nodeHandle_->create_subscription<trans::msg::TouchSensor>(
            topic_prefix + touch_sensor_topic, qos,
            std::bind(&StateEstimationLKF::touchCallback, this,
                      std::placeholders::_1));

    std::string joints_topic =
        config_["global"]["topic_names"]["joints_state"].as<std::string>();
    joints_state_subscription_ =
        nodeHandle_->create_subscription<sensor_msgs::msg::JointState>(
            topic_prefix + joints_topic, qos,
            std::bind(&StateEstimationLKF::jointCallback, this,
                      std::placeholders::_1));

    std::string odom_topic =
        config_["global"]["topic_names"]["odom"].as<std::string>();
    odom_subscription_ =
        nodeHandle_->create_subscription<nav_msgs::msg::Odometry>(
            topic_prefix + odom_topic, qos,
            std::bind(&StateEstimationLKF::odomCallback, this,
                      std::placeholders::_1));
  }

  inner_loop_thread_ = std::thread(&StateEstimationLKF::innerLoop, this);
  run_.push(true);
  setup();
}

StateEstimationLKF::~StateEstimationLKF() {
  run_.push(false);
  inner_loop_thread_.join();
}

void StateEstimationLKF::setup() {
  x_est.setZero();
  x_est.z() = 0.1;
  ps_.setZero();
  vs_.setZero();

  A_.setIdentity();
  A_.block<3, 3>(0, 3).diagonal().fill(dt_);

  B_.setZero();
  B_.block<3, 3>(3, 0).diagonal().fill(dt_);

  matrix_t C1 = matrix_t::Zero(3, 6);
  C1.leftCols(3).setIdentity();
  matrix_t C2 = matrix_t::Zero(3, 6);
  C2.rightCols(3).setIdentity();

  C_.setZero();
  C_.block<3, 6>(0, 0) = C1;
  C_.block<3, 6>(3, 0) = C1;
  C_.block<3, 6>(6, 0) = C1;
  C_.block<3, 6>(9, 0) = C1;
  C_.block<12, 12>(0, 6).diagonal().fill(-1.0);
  C_.block<3, 6>(12, 0) = C2;
  C_.block<3, 6>(15, 0) = C2;
  C_.block<3, 6>(18, 0) = C2;
  C_.block<3, 6>(21, 0) = C2;
  C_(24, 8) = 1.0;
  C_(25, 11) = 1.0;
  C_(26, 14) = 1.0;
  C_(27, 17) = 1.0;

  Sigma_.setZero();
  Sigma_.diagonal().fill(100.0);
  Q0_.setIdentity();
  Q0_.topLeftCorner(3, 3).diagonal().fill(dt_ / 20.0);
  Q0_.block<3, 3>(3, 3).diagonal().fill(dt_ * 9.81 / 20.0);
  Q0_.bottomRightCorner(12, 12).diagonal().fill(dt_);
  R0_.setIdentity();
}

void StateEstimationLKF::setContactFlag(vector<bool> flag) { cflag_ = flag; }

void StateEstimationLKF::angularMotionEstimate(
    const sensor_msgs::msg::Imu &imu_data, std::shared_ptr<vector_t> qpos,
    std::shared_ptr<vector_t> qvel) {
  const auto &quat_imu = imu_data.orientation;
  Eigen::Quaternion<scalar_t> quat(quat_imu.w, quat_imu.x, quat_imu.y,
                                   quat_imu.z);
  qpos->segment(3, 4) << quat_imu.x, quat_imu.y, quat_imu.z, quat_imu.w;
  qvel->segment(3, 3) << imu_data.angular_velocity.x,
      imu_data.angular_velocity.y, imu_data.angular_velocity.z;
}

void StateEstimationLKF::linearMotionEstimate(
    const sensor_msgs::msg::Imu &imu_data, std::shared_ptr<vector_t> qpos,
    std::shared_ptr<vector_t> qvel) {
  Eigen::Quaternion<scalar_t> quat(
      imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y,
      imu_data.orientation.z);
  vector3_t b_acc(imu_data.linear_acceleration.x,
                  imu_data.linear_acceleration.y,
                  imu_data.linear_acceleration.z);
  vector3_t w_acc = quat.toRotationMatrix() * b_acc;
  vector3_t g(0, 0, -9.81);
  w_acc = w_acc + g;

  scalar_t noise_pimu = 0.02;
  scalar_t noise_vimu = 0.02;
  scalar_t noise_pfoot = 0.002;
  scalar_t noise_pimu_rel_foot = 0.001;
  scalar_t noise_vimu_rel_foot = 0.02;
  scalar_t noise_zfoot = 0.001;

  matrix_t Q = matrix_t::Identity(18, 18);
  Q.topLeftCorner(3, 3) = Q0_.topLeftCorner(3, 3) * noise_pimu;
  Q.block<3, 3>(3, 3) = Q0_.block<3, 3>(3, 3) * noise_vimu;
  Q.block<12, 12>(6, 6) = Q0_.block<12, 12>(6, 6) * noise_pfoot;

  matrix_t R = matrix_t::Identity(28, 28);
  R.topLeftCorner(12, 12) = R0_.topLeftCorner(12, 12) * noise_pimu_rel_foot;
  R.block<12, 12>(12, 12) = R0_.block<12, 12>(12, 12) * noise_vimu_rel_foot;
  R.block<4, 4>(24, 24) = R0_.block<4, 4>(24, 24) * noise_zfoot;

  size_t q_idx = 0;
  size_t idx1 = 0;
  size_t idx2 = 0;

  vector4_t pzs = vector4_t::Zero();
  vector3_t p0, v0;
  p0 << x_est[0], x_est[1], x_est[2];
  v0 << x_est[3], x_est[4], x_est[5];

  pinocchioInterface_ptr->updateRobotState(*qpos, *qvel);
  const auto touch_sensor_data = touch_msg_buffer.get()->value;

  bool release_ = (abs(w_acc.z()) < 1.0);
  for (auto &data : touch_sensor_data) {
    release_ &= (data < 2.0);
=======
namespace clear
{

  StateEstimationLKF::StateEstimationLKF(Node::SharedPtr nodeHandle)
      : nodeHandle_(nodeHandle)
  {

    const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                         .get_parameter_value()
                                         .get<std::string>();

    auto config_ = YAML::LoadFile(config_file_);

    robot_name = config_["model"]["name"].as<std::string>();
    RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "robot_name: %s",
                robot_name.c_str());

    std::string model_package = config_["model"]["package"].as<std::string>();
    std::string urdf =
        ament_index_cpp::get_package_share_directory(model_package) +
        config_["model"]["urdf"].as<std::string>();
    RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "model file: %s",
                urdf.c_str());

    pinocchioInterface_ptr = std::make_unique<PinocchioInterface>(urdf.c_str());
    foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
    dt_ = config_["estimation"]["dt"].as<scalar_t>();
    use_odom_ = config_["estimation"]["use_odom"].as<bool>();

    RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "dt: %f", dt_);
    RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "use odom: %s",
                use_odom_ ? "true" : "false");
    for (const auto &name : foot_names)
    {
      RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "foot name: %s",
                  name.c_str());
      cflag_.emplace_back(true);
    }

    bool hardware_ = config_["estimation"]["hardware"].as<bool>();

    if (!hardware_)
    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
      std::string topic_prefix =
          config_["global"]["topic_prefix"].as<std::string>();
      std::string imu_topic =
          config_["global"]["topic_names"]["imu"].as<std::string>();
      imu_subscription_ = nodeHandle_->create_subscription<sensor_msgs::msg::Imu>(
          topic_prefix + imu_topic, qos,
          std::bind(&StateEstimationLKF::imuCallback, this,
                    std::placeholders::_1));

      std::string touch_sensor_topic =
          config_["global"]["topic_names"]["touch_sensor"].as<std::string>();
      touch_subscription_ =
          nodeHandle_->create_subscription<trans::msg::TouchSensor>(
              topic_prefix + touch_sensor_topic, qos,
              std::bind(&StateEstimationLKF::touchCallback, this,
                        std::placeholders::_1));

      std::string joints_topic =
          config_["global"]["topic_names"]["joints_state"].as<std::string>();
      joints_state_subscription_ =
          nodeHandle_->create_subscription<sensor_msgs::msg::JointState>(
              topic_prefix + joints_topic, qos,
              std::bind(&StateEstimationLKF::jointCallback, this,
                        std::placeholders::_1));

      std::string odom_topic =
          config_["global"]["topic_names"]["odom"].as<std::string>();
      odom_subscription_ =
          nodeHandle_->create_subscription<nav_msgs::msg::Odometry>(
              topic_prefix + odom_topic, qos,
              std::bind(&StateEstimationLKF::odomCallback, this,
                        std::placeholders::_1));
    }

    inner_loop_thread_ = std::thread(&StateEstimationLKF::innerLoop, this);
    run_.push(true);
    setup();
  }

  StateEstimationLKF::~StateEstimationLKF()
  {
    run_.push(false);
    inner_loop_thread_.join();
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42
  }

  void StateEstimationLKF::setup()
  {
    x_est.setZero();
    x_est.z() = 0.1;
    ps_.setZero();
    vs_.setZero();

    A_.setIdentity();
    A_.block<3, 3>(0, 3).diagonal().fill(dt_);

    B_.setZero();
    B_.block<3, 3>(3, 0).diagonal().fill(dt_);

    matrix_t C1 = matrix_t::Zero(3, 6);
    C1.leftCols(3).setIdentity();
    matrix_t C2 = matrix_t::Zero(3, 6);
    C2.rightCols(3).setIdentity();

    C_.setZero();
    C_.block<3, 6>(0, 0) = C1;
    C_.block<3, 6>(3, 0) = C1;
    C_.block<3, 6>(6, 0) = C1;
    C_.block<3, 6>(9, 0) = C1;
    C_.block<12, 12>(0, 6).diagonal().fill(-1.0);
    C_.block<3, 6>(12, 0) = C2;
    C_.block<3, 6>(15, 0) = C2;
    C_.block<3, 6>(18, 0) = C2;
    C_.block<3, 6>(21, 0) = C2;
    C_(24, 8) = 1.0;
    C_(25, 11) = 1.0;
    C_(26, 14) = 1.0;
    C_(27, 17) = 1.0;

    Sigma_.setZero();
    Sigma_.diagonal().fill(100.0);
    Q0_.setIdentity();
    Q0_.topLeftCorner(3, 3).diagonal().fill(dt_ / 20.0);
    Q0_.block<3, 3>(3, 3).diagonal().fill(dt_ * 9.81 / 20.0);
    Q0_.bottomRightCorner(12, 12).diagonal().fill(dt_);
    R0_.setIdentity();
  }

  void StateEstimationLKF::setContactFlag(vector<bool> flag) { cflag_ = flag; }

  void StateEstimationLKF::angularMotionEstimate(
      const sensor_msgs::msg::Imu &imu_data, std::shared_ptr<vector_t> qpos,
      std::shared_ptr<vector_t> qvel)
  {
    const auto &quat_imu = imu_data.orientation;
    Eigen::Quaternion<scalar_t> quat(quat_imu.w, quat_imu.x, quat_imu.y,
                                     quat_imu.z);
    qpos->segment(3, 4) << quat_imu.x, quat_imu.y, quat_imu.z, quat_imu.w;
    qvel->segment(3, 3) << imu_data.angular_velocity.x,
        imu_data.angular_velocity.y, imu_data.angular_velocity.z;
  }
<<<<<<< HEAD
  qpos->head(3) = x_est.head(3);
  qvel->head(3) = quat.toRotationMatrix().transpose() * x_est.segment(3, 3);
}

void StateEstimationLKF::innerLoop() {
  rclcpp::Rate loop_rate(1.0 / dt_);
  std::fstream save_motor_tau(log_dir + "/motor_tau_log.txt",
                              std::ios::ate | std::ios::out);

  const scalar_t t0 = nodeHandle_->now().seconds();
  while (rclcpp::ok() && run_.get()) {
    if (imu_msg_buffer.get().get() == nullptr ||
        touch_msg_buffer.get().get() == nullptr ||
        joint_state_msg_buffer.get().get() == nullptr ||
        (use_odom_ && odom_msg_buffer.get().get() == nullptr)) {
      continue;
    }
    // do not delete these three lines
    sensor_msgs::msg::Imu::SharedPtr imu_msg = imu_msg_buffer.get();
    sensor_msgs::msg::JointState::SharedPtr joint_state_msg =
        joint_state_msg_buffer.get();
    nav_msgs::msg::Odometry::SharedPtr odom_msg = odom_msg_buffer.get();

    const pin::Model &model_ = pinocchioInterface_ptr->getModel();
    std::shared_ptr<vector_t> qpos_ptr_ = std::make_shared<vector_t>(model_.nq);
    qpos_ptr_->setZero();
    std::shared_ptr<vector_t> qvel_ptr_ = std::make_shared<vector_t>(model_.nv);
    qvel_ptr_->setZero();

    vector_t tau_est = vector_t::Zero(joint_state_msg->name.size());
    for (size_t k = 0; k < joint_state_msg->name.size(); k++) {
      if (static_cast<int>(k) < model_.njoints &&
          model_.existJointName(joint_state_msg->name[k])) {
        pin::Index id = model_.getJointId(joint_state_msg->name[k]) - 2;
        (*qpos_ptr_)[id + 7] = joint_state_msg->position[k];
        (*qvel_ptr_)[id + 6] = joint_state_msg->velocity[k];
        tau_est[id] = joint_state_msg->effort[k];
=======

  void StateEstimationLKF::linearMotionEstimate(
      const sensor_msgs::msg::Imu &imu_data, std::shared_ptr<vector_t> qpos,
      std::shared_ptr<vector_t> qvel)
  {
    Eigen::Quaternion<scalar_t> quat(
        imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y,
        imu_data.orientation.z);
    vector3_t b_acc(imu_data.linear_acceleration.x,
                    imu_data.linear_acceleration.y,
                    imu_data.linear_acceleration.z);
    vector3_t w_acc = quat.toRotationMatrix() * b_acc;
    vector3_t g(0, 0, -9.81);
    w_acc = w_acc + g;

    scalar_t noise_pimu = 0.02;
    scalar_t noise_vimu = 0.02;
    scalar_t noise_pfoot = 0.002;
    scalar_t noise_pimu_rel_foot = 0.001;
    scalar_t noise_vimu_rel_foot = 0.02;
    scalar_t noise_zfoot = 0.001;

    matrix_t Q = matrix_t::Identity(18, 18);
    Q.topLeftCorner(3, 3) = Q0_.topLeftCorner(3, 3) * noise_pimu;
    Q.block<3, 3>(3, 3) = Q0_.block<3, 3>(3, 3) * noise_vimu;
    Q.block<12, 12>(6, 6) = Q0_.block<12, 12>(6, 6) * noise_pfoot;

    matrix_t R = matrix_t::Identity(28, 28);
    R.topLeftCorner(12, 12) = R0_.topLeftCorner(12, 12) * noise_pimu_rel_foot;
    R.block<12, 12>(12, 12) = R0_.block<12, 12>(12, 12) * noise_vimu_rel_foot;
    R.block<4, 4>(24, 24) = R0_.block<4, 4>(24, 24) * noise_zfoot;

    size_t q_idx = 0;
    size_t idx1 = 0;
    size_t idx2 = 0;

    vector4_t pzs = vector4_t::Zero();
    vector3_t p0, v0;
    p0 << x_est[0], x_est[1], x_est[2];
    v0 << x_est[3], x_est[4], x_est[5];

    pinocchioInterface_ptr->updateRobotState(*qpos, *qvel);
    const auto touch_sensor_data = touch_msg_buffer.get()->value;

    bool release_ = (abs(w_acc.z()) < 1.0);
    for (auto &data : touch_sensor_data)
    {
      release_ &= (data < 2.0);
    }

    for (size_t i = 0; i < foot_names.size(); i++)
    {
      auto pose = pinocchioInterface_ptr->getFramePose(foot_names[i]);
      auto v =
          pinocchioInterface_ptr->getFrame6dVel_localWorldAligned(foot_names[i]);
      vector3_t p_f = pose.translation();
      p_f.z() -= 0.02; // the foot radius, be careful
      vector3_t v_f = v.linear();

      int i1 = 3 * i;
      q_idx = 6 + i1;
      idx1 = 12 + i1;
      idx2 = 24 + i;

      scalar_t trust;
      if (touch_sensor_data[i] > 2.0 || release_)
      {
        trust = 1.0;
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42
      }
      else
      {
        trust = std::max(touch_sensor_data[i] / 2.0, 0.0);
      }
      scalar_t high_suspect_number = 500.0;

      Q.block<3, 3>(q_idx, q_idx) = (1.0 + (1.0 - trust) * high_suspect_number) *
                                    Q.block<3, 3>(q_idx, q_idx);
      R.block<3, 3>(idx1, idx1) =
          (1.0 + (1.0 - trust) * high_suspect_number) * R.block<3, 3>(idx1, idx1);
      R(idx2, idx2) = (1.0 + (1.0 - trust) * high_suspect_number) * R(idx2, idx2);
      ps_.segment(i1, 3) = -p_f;
      pzs(i) = (1.0 - trust) * (p0(2) + p_f(2));
      vs_.segment(i1, 3) = (1.0 - trust) * v0 + trust * (-v_f);
    }
    save_motor_tau << rclcpp::Time(joint_state_msg->header.stamp).seconds()
                   << " " << tau_est.transpose() << "\n";

<<<<<<< HEAD
    if (use_odom_) {
      const auto &orientation = odom_msg->pose.pose.orientation;
      const auto &position = odom_msg->pose.pose.position;
      const auto &vel = odom_msg->twist.twist.linear;
      const auto &ang_vel = odom_msg->twist.twist.angular;

      Eigen::Quaternion<scalar_t> quat(orientation.w, orientation.x,
                                       orientation.y, orientation.z);
      matrix3_t rot = quat.toRotationMatrix();
      (*qpos_ptr_).head(3) << position.x, position.y, position.z;
      (*qvel_ptr_).head(3) << vel.x, vel.y, vel.z;
      (*qvel_ptr_).head(3) = rot.transpose() * (*qvel_ptr_).head(3);

      (*qpos_ptr_).segment(3, 4) << orientation.x, orientation.y, orientation.z,
          orientation.w;
      (*qvel_ptr_).segment(3, 3) << ang_vel.x, ang_vel.y, ang_vel.z;
    } else {
      // const auto touch_sensor_data = touch_msg_buffer.get()->value;
      // for (auto &data : touch_sensor_data) {
      //   RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"),
      //   "touch_sensor_data: %f", data);
      // }
      angularMotionEstimate(*imu_msg, qpos_ptr_, qvel_ptr_);
      linearMotionEstimate(*imu_msg, qpos_ptr_, qvel_ptr_);
=======
    Eigen::Matrix<scalar_t, 28, 1> y;
    y << ps_, vs_, pzs;
    vector_t x_pred = A_ * x_est + B_ * w_acc;

    matrix_t Sigma_bar = A_ * Sigma_ * A_.transpose() + Q;
    matrix_t S = C_ * Sigma_bar * C_.transpose() + R;
    vector_t correct = S.lu().solve(y - C_ * x_pred);
    x_est = x_pred + Sigma_bar * C_.transpose() * correct;
    matrix_t SC_ = S.lu().solve(C_);
    Sigma_ = (matrix_t::Identity(18, 18) - Sigma_bar * C_.transpose() * SC_) *
             Sigma_bar;
    Sigma_ = 0.5 * (Sigma_ + Sigma_.transpose());
    if (Sigma_.topLeftCorner(2, 2).determinant() > 1e-6)
    {
      Sigma_.topRightCorner(2, 16).setZero();
      Sigma_.bottomLeftCorner(16, 2).setZero();
      Sigma_.topLeftCorner(2, 2) *= 0.1;
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42
    }
    qpos->head(3) = x_est.head(3);
    qvel->head(3) = quat.toRotationMatrix().transpose() * x_est.segment(3, 3);
  }

<<<<<<< HEAD
    if (nodeHandle_->now().seconds() - t0 > 0.3) {
      qpos_ptr_buffer.push(qpos_ptr_);
      qvel_ptr_buffer.push(qvel_ptr_);
    }

    /* RCLCPP_INFO_STREAM(rclcpp::get_logger("StateEstimationLKF"),
                       "qpos: " << (*qpos_ptr_).transpose() << "\n");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("StateEstimationLKF"),
                       "qvel: " << (*qvel_ptr_).transpose() << "\n"); */
=======
  void StateEstimationLKF::innerLoop()
  {
    rclcpp::Rate loop_rate(1.0 / dt_);
    std::fstream save_motor_tau("motor_tau_log.txt", std::ios::ate | std::ios::out);

    const scalar_t t0 = nodeHandle_->now().seconds();
    while (rclcpp::ok() && run_.get())
    {
      if (imu_msg_buffer.get().get() == nullptr ||
          touch_msg_buffer.get().get() == nullptr ||
          joint_state_msg_buffer.get().get() == nullptr ||
          (use_odom_ && odom_msg_buffer.get().get() == nullptr))
      {
        continue;
      }
      // do not delete these three lines
      sensor_msgs::msg::Imu::SharedPtr imu_msg = imu_msg_buffer.get();
      sensor_msgs::msg::JointState::SharedPtr joint_state_msg =
          joint_state_msg_buffer.get();
      nav_msgs::msg::Odometry::SharedPtr odom_msg = odom_msg_buffer.get();

      const pin::Model &model_ = pinocchioInterface_ptr->getModel();
      std::shared_ptr<vector_t> qpos_ptr_ = std::make_shared<vector_t>(model_.nq);
      qpos_ptr_->setZero();
      std::shared_ptr<vector_t> qvel_ptr_ = std::make_shared<vector_t>(model_.nv);
      qvel_ptr_->setZero();

      vector_t tau_est = vector_t::Zero(joint_state_msg->name.size());
      for (size_t k = 0; k < joint_state_msg->name.size(); k++)
      {
        if (static_cast<int>(k) < model_.njoints &&
            model_.existJointName(joint_state_msg->name[k]))
        {
          pin::Index id = model_.getJointId(joint_state_msg->name[k]) - 2;
          (*qpos_ptr_)[id + 7] = joint_state_msg->position[k];
          (*qvel_ptr_)[id + 6] = joint_state_msg->velocity[k];
          tau_est[id] = joint_state_msg->effort[k];
        }
      }
      save_motor_tau << rclcpp::Time(joint_state_msg->header.stamp).seconds() << " " << tau_est.transpose() << "\n";

      if (use_odom_)
      {
        const auto &orientation = odom_msg->pose.pose.orientation;
        const auto &position = odom_msg->pose.pose.position;
        const auto &vel = odom_msg->twist.twist.linear;
        const auto &ang_vel = odom_msg->twist.twist.angular;

        Eigen::Quaternion<scalar_t> quat(orientation.w, orientation.x,
                                         orientation.y, orientation.z);
        matrix3_t rot = quat.toRotationMatrix();
        (*qpos_ptr_).head(3) << position.x, position.y, position.z;
        (*qvel_ptr_).head(3) << vel.x, vel.y, vel.z;
        (*qvel_ptr_).head(3) = rot.transpose() * (*qvel_ptr_).head(3);

        (*qpos_ptr_).segment(3, 4) << orientation.x, orientation.y, orientation.z,
            orientation.w;
        (*qvel_ptr_).segment(3, 3) << ang_vel.x, ang_vel.y, ang_vel.z;
      }
      else
      {
        // const auto touch_sensor_data = touch_msg_buffer.get()->value;
        // for (auto &data : touch_sensor_data) {
        //   RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"),
        //   "touch_sensor_data: %f", data);
        // }
        angularMotionEstimate(*imu_msg, qpos_ptr_, qvel_ptr_);
        linearMotionEstimate(*imu_msg, qpos_ptr_, qvel_ptr_);
      }

      if (nodeHandle_->now().seconds() - t0 > 0.3)
      {
        qpos_ptr_buffer.push(qpos_ptr_);
        qvel_ptr_buffer.push(qvel_ptr_);
      }

      /* RCLCPP_INFO_STREAM(rclcpp::get_logger("StateEstimationLKF"),
                         "qpos: " << (*qpos_ptr_).transpose() << "\n");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("StateEstimationLKF"),
                         "qvel: " << (*qvel_ptr_).transpose() << "\n"); */
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42

      loop_rate.sleep();
    }
    save_motor_tau.close();
  }
<<<<<<< HEAD
  save_motor_tau.close();
}
=======
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42

  std::shared_ptr<vector_t> StateEstimationLKF::getQpos()
  {
    return qpos_ptr_buffer.get();
  }

  std::shared_ptr<vector_t> StateEstimationLKF::getQvel()
  {
    return qvel_ptr_buffer.get();
  }

<<<<<<< HEAD
void StateEstimationLKF::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) const {
  if (msg->header.frame_id == robot_name) {
    imu_msg_buffer.push(msg);
=======
  void StateEstimationLKF::imuCallback(
      const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    if (msg->header.frame_id == robot_name)
    {
      imu_msg_buffer.push(msg);
    }
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42
  }

<<<<<<< HEAD
void StateEstimationLKF::touchCallback(
    const trans::msg::TouchSensor::SharedPtr msg) const {
  if (msg->header.frame_id == robot_name) {
    touch_msg_buffer.push(msg);
=======
  void StateEstimationLKF::touchCallback(
      const trans::msg::TouchSensor::SharedPtr msg) const
  {
    if (msg->header.frame_id == robot_name)
    {
      touch_msg_buffer.push(msg);
    }
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42
  }

<<<<<<< HEAD
void StateEstimationLKF::jointCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) const {
  if (msg->header.frame_id == robot_name) {
    joint_state_msg_buffer.push(msg);
=======
  void StateEstimationLKF::jointCallback(
      const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
    if (msg->header.frame_id == robot_name)
    {
      joint_state_msg_buffer.push(msg);
    }
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42
  }

<<<<<<< HEAD
void StateEstimationLKF::setImuMsg(sensor_msgs::msg::Imu::SharedPtr msg) {
  if (msg->header.frame_id == robot_name) {
    imu_msg_buffer.push(msg);
=======
  void StateEstimationLKF::setImuMsg(sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (msg->header.frame_id == robot_name)
    {
      imu_msg_buffer.push(msg);
    }
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42
  }

<<<<<<< HEAD
void StateEstimationLKF::setTouchMsg(trans::msg::TouchSensor::SharedPtr msg) {
  if (msg->header.frame_id == robot_name) {
    touch_msg_buffer.push(msg);
=======
  void StateEstimationLKF::setTouchMsg(trans::msg::TouchSensor::SharedPtr msg)
  {
    if (msg->header.frame_id == robot_name)
    {
      touch_msg_buffer.push(msg);
    }
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42
  }

<<<<<<< HEAD
void StateEstimationLKF::setJointsMsg(
    sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->header.frame_id == robot_name) {
    joint_state_msg_buffer.push(msg);
=======
  void StateEstimationLKF::setJointsMsg(
      sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->header.frame_id == robot_name)
    {
      joint_state_msg_buffer.push(msg);
    }
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42
  }

<<<<<<< HEAD
void StateEstimationLKF::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) const {
  if (msg->header.frame_id == robot_name) {
    odom_msg_buffer.push(msg);
=======
  void StateEstimationLKF::odomCallback(
      const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    if (msg->header.frame_id == robot_name)
    {
      odom_msg_buffer.push(msg);
    }
>>>>>>> 77dc56d93f6ad7c572b7e85c1990f7f32b525a42
  }

} // namespace clear