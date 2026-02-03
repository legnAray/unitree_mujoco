#include "unitree_legged_bridge.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace unitree_legged_bridge {
namespace uls = UNITREE_LEGGED_SDK;

static constexpr int kBridgeHz = 1000;

UnitreeLeggedBridge::UnitreeLeggedBridge(mjModel *model, mjData *data)
    : mj_model_(model), mj_data_(data) {
  if (!mj_model_ || !mj_data_) {
    std::cerr << "UnitreeLeggedBridge: invalid mujoco pointers." << std::endl;
    return;
  }

  checkSensors();

  // Emulate robot UDP: bind to server port and send LowState to client port.
  std::string target_ip = uls::UDP_SERVER_IP_BASIC;
  long server_port = uls::UDP_SERVER_PORT;
  long client_port = uls::UDP_CLIENT_PORT;
  if (const char *env = std::getenv("STEPIT_UNITREE_UDP_IP")) target_ip = env;
  if (const char *env = std::getenv("STEPIT_UNITREE_UDP_SERVER_PORT")) server_port = std::strtol(env, nullptr, 10);
  if (const char *env = std::getenv("STEPIT_UNITREE_UDP_CLIENT_PORT")) client_port = std::strtol(env, nullptr, 10);

  sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) {
    std::cerr << "UnitreeLeggedBridge: failed to create UDP socket (" << std::strerror(errno) << ")." << std::endl;
    return;
  }

  int flags = ::fcntl(sock_fd_, F_GETFL, 0);
  ::fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);

  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(static_cast<uint16_t>(server_port));
  server_addr_.sin_addr.s_addr = INADDR_ANY;

  if (::bind(sock_fd_, reinterpret_cast<sockaddr *>(&server_addr_), sizeof(server_addr_)) < 0) {
    std::cerr << "UnitreeLeggedBridge: failed to bind UDP socket (" << std::strerror(errno) << ")." << std::endl;
    ::close(sock_fd_);
    sock_fd_ = -1;
    return;
  }

  std::memset(&client_addr_, 0, sizeof(client_addr_));
  client_addr_.sin_family = AF_INET;
  client_addr_.sin_port = htons(static_cast<uint16_t>(client_port));
  ::inet_pton(AF_INET, target_ip.c_str(), &client_addr_.sin_addr);
  if (sock_fd_ < 0) return;
  std::memset(&low_cmd_, 0, sizeof(low_cmd_));
  low_cmd_.levelFlag = uls::LOWLEVEL;
  for (auto &motor_cmd : low_cmd_.motorCmd) {
    motor_cmd.mode = 0x0A;
    motor_cmd.q    = uls::PosStopF;
    motor_cmd.dq   = uls::VelStopF;
    motor_cmd.Kp   = 0.0f;
    motor_cmd.Kd   = 0.0f;
    motor_cmd.tau  = 0.0f;
  }

  std::memset(&low_state_, 0, sizeof(low_state_));
  low_state_.levelFlag = uls::LOWLEVEL;
}

UnitreeLeggedBridge::~UnitreeLeggedBridge() { stop(); }

void UnitreeLeggedBridge::start() {
  if (running_) return;
  running_ = true;
  thread_ = std::thread([this]() { run(); });
}

void UnitreeLeggedBridge::stop() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
}

void UnitreeLeggedBridge::run() {
  using clock = std::chrono::steady_clock;
  auto next_tick = clock::now();
  const auto period = std::chrono::microseconds(1000000 / kBridgeHz);

  while (running_) {
    next_tick += period;

    recvLowCmd();
    applyLowCmd();
    updateLowState();
    sendLowState();

    std::this_thread::sleep_until(next_tick);
  }
}

void UnitreeLeggedBridge::checkSensors() {
  num_motor_ = mj_model_->nu;

  int sensor_id = -1;
  sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_quat");
  if (sensor_id >= 0) imu_quat_adr_ = mj_model_->sensor_adr[sensor_id];

  sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_gyro");
  if (sensor_id >= 0) imu_gyro_adr_ = mj_model_->sensor_adr[sensor_id];

  sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, "imu_acc");
  if (sensor_id >= 0) imu_acc_adr_ = mj_model_->sensor_adr[sensor_id];

  if (param::config.print_scene_information == 1) {
    std::cout << "[UnitreeLeggedBridge] motor count: " << num_motor_ << std::endl;
  }
}

void UnitreeLeggedBridge::recvLowCmd() {
  if (sock_fd_ < 0) return;
  uls::LowCmd tmp{};
  sockaddr_in src_addr{};
  socklen_t src_len = sizeof(src_addr);
  const ssize_t n = ::recvfrom(sock_fd_, &tmp, sizeof(tmp), MSG_DONTWAIT,
                               reinterpret_cast<sockaddr *>(&src_addr), &src_len);
  if (n == static_cast<ssize_t>(sizeof(tmp))) {
    low_cmd_ = tmp;
    client_addr_ = src_addr;
    client_addr_len_ = src_len;
    has_client_ = true;
  }
}

void UnitreeLeggedBridge::applyLowCmd() {
  if (!mj_data_) return;
  const int count = std::min<int>(num_motor_, static_cast<int>(low_cmd_.motorCmd.size()));
  for (int i = 0; i < count; ++i) {
    const auto &cmd = low_cmd_.motorCmd[i];
    const double q  = mj_data_->sensordata[i];
    const double dq = mj_data_->sensordata[i + num_motor_];
    mj_data_->ctrl[i] = cmd.tau + cmd.Kp * (cmd.q - q) + cmd.Kd * (cmd.dq - dq);
  }
}

void UnitreeLeggedBridge::updateLowState() {
  if (!mj_data_) return;
  const int count = std::min<int>(num_motor_, static_cast<int>(low_state_.motorState.size()));
  for (int i = 0; i < count; ++i) {
    auto &state = low_state_.motorState[i];
    state.q     = mj_data_->sensordata[i];
    state.dq    = mj_data_->sensordata[i + num_motor_];
    state.tauEst = mj_data_->sensordata[i + 2 * num_motor_];
    state.mode  = 0x0A;
  }

  if (imu_quat_adr_ >= 0) {
    auto &imu = low_state_.imu;
    imu.quaternion[0] = mj_data_->sensordata[imu_quat_adr_ + 0];
    imu.quaternion[1] = mj_data_->sensordata[imu_quat_adr_ + 1];
    imu.quaternion[2] = mj_data_->sensordata[imu_quat_adr_ + 2];
    imu.quaternion[3] = mj_data_->sensordata[imu_quat_adr_ + 3];

    const double w = imu.quaternion[0];
    const double x = imu.quaternion[1];
    const double y = imu.quaternion[2];
    const double z = imu.quaternion[3];
    imu.rpy[0] = static_cast<float>(std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)));
    imu.rpy[1] = static_cast<float>(std::asin(2.0 * (w * y - z * x)));
    imu.rpy[2] = static_cast<float>(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
  }

  if (imu_gyro_adr_ >= 0) {
    auto &imu = low_state_.imu;
    imu.gyroscope[0] = mj_data_->sensordata[imu_gyro_adr_ + 0];
    imu.gyroscope[1] = mj_data_->sensordata[imu_gyro_adr_ + 1];
    imu.gyroscope[2] = mj_data_->sensordata[imu_gyro_adr_ + 2];
  }

  if (imu_acc_adr_ >= 0) {
    auto &imu = low_state_.imu;
    imu.accelerometer[0] = mj_data_->sensordata[imu_acc_adr_ + 0];
    imu.accelerometer[1] = mj_data_->sensordata[imu_acc_adr_ + 1];
    imu.accelerometer[2] = mj_data_->sensordata[imu_acc_adr_ + 2];
  }

  uint32_t tick = 0;
  if (mj_data_) {
    tick = static_cast<uint32_t>(std::llround(mj_data_->time * 1000.0));
  }
  if (tick == 0) {
    tick = ++tick_;
  } else {
    tick_ = tick;
  }
  low_state_.tick = tick;
  low_state_.crc  = uls::crc32(reinterpret_cast<uint32_t *>(&low_state_), (sizeof(low_state_) >> 2) - 1);
}

void UnitreeLeggedBridge::sendLowState() {
  if (sock_fd_ < 0) return;
  const sockaddr_in &dst = has_client_ ? client_addr_ : client_addr_;
  const socklen_t dst_len = has_client_ ? client_addr_len_ : sizeof(client_addr_);
  ::sendto(sock_fd_, &low_state_, sizeof(low_state_), 0,
           reinterpret_cast<const sockaddr *>(&dst), dst_len);
}
}  // namespace unitree_legged_bridge
