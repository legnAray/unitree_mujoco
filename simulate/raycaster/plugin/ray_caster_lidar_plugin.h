#pragma once

#include "ray_plugin.h"
#include "RayCasterLidar.h"

namespace mujoco::plugin::sensor {

class RayCasterLidarPlugin : public RayPlugin {
public:
  static RayCasterLidarPlugin *Create(const mjModel *m, mjData *d,
                                       int instance);
  RayCasterLidarPlugin(RayCasterLidarPlugin &&) = default;
  ~RayCasterLidarPlugin() = default;
  RayCasterLidarCfg cfg;

  static void RegisterPlugin();

  static constexpr std::array<const char *, 5> ray_attributes = {
      "fov_h", "fov_v", "size", "dis_range", "iteration_order"};

private:
  RayCasterLidarPlugin(const mjModel *m, mjData *d, int instance);
  std::vector<mjtNum> distance_;
};

} // namespace mujoco::plugin::sensor
