
#pragma once
#include "livox_points_plugin.h"

namespace gazebo {

class PluginLivoxCustom : public LivoxPointsPlugin {
 public:
  PluginLivoxCustom(): LivoxPointsPlugin(){};
  ~PluginLivoxCustom() override {};

 protected:
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) override ;

  void OnNewLaserScans() override ;
};

}