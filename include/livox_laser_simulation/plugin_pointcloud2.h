
#pragma once
#include "livox_points_plugin.h"

namespace gazebo {

class PluginPointCloud2 : public LivoxPointsPlugin {
 public:
  PluginPointCloud2(): LivoxPointsPlugin(){};
  ~PluginPointCloud2() override {};

 protected:
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) override ;

  void OnNewLaserScans() override ;
};

}