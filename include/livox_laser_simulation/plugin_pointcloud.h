
#pragma once
#include "livox_points_plugin.h"

namespace gazebo {

class PluginPointCloud : public LivoxPointsPlugin {
 public:
  PluginPointCloud(): LivoxPointsPlugin(){};
  ~PluginPointCloud() override {};

 protected:
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) override ;

  void OnNewLaserScans() override ;
};

}