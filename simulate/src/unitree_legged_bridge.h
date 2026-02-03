#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

#include <netinet/in.h>

#include <mujoco/mujoco.h>
#include <unitree_legged_sdk/unitree_legged_sdk.h>

#include "param.h"

// Forward declaration for CRC function implemented in unitree_legged_sdk library.
namespace UNITREE_LEGGED_SDK {
uint32_t crc32(uint32_t *ptr, uint32_t len);
}

namespace unitree_legged_bridge {
namespace uls = UNITREE_LEGGED_SDK;

class UnitreeLeggedBridge {
 public:
  UnitreeLeggedBridge(mjModel *model, mjData *data);
  ~UnitreeLeggedBridge();

  void start();
  void stop();

 private:
  void run();
  void checkSensors();
  void recvLowCmd();
  void applyLowCmd();
  void updateLowState();
  void sendLowState();

  mjModel *mj_model_{nullptr};
  mjData *mj_data_{nullptr};

  int num_motor_{0};
  int imu_quat_adr_{-1};
  int imu_gyro_adr_{-1};
  int imu_acc_adr_{-1};

  int sock_fd_{-1};
  sockaddr_in server_addr_{};
  sockaddr_in client_addr_{};
  socklen_t client_addr_len_{sizeof(sockaddr_in)};
  bool has_client_{false};
  uls::LowCmd low_cmd_{};
  uls::LowState low_state_{};

  std::atomic<bool> running_{false};
  std::thread thread_;
  uint32_t tick_{0};
};
}  // namespace unitree_legged_bridge
