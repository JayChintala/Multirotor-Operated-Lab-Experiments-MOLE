// Compatibility layer: build the same sources against Gazebo Harmonic
// (gz-sim8, ROS 2 Jazzy) or Gazebo Fortress (ignition-gazebo6, ROS 2 Humble).
// CMakeLists.txt defines EMSD_GZ_HARMONIC=1 when it finds gz-sim8.
#pragma once

#if EMSD_GZ_HARMONIC
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
namespace emsd
{
namespace gzsim = gz::sim;
namespace gzmath = gz::math;
namespace gzcommon = gz::common;
namespace gzplugin = gz::plugin;
}
#define EMSD_ADD_PLUGIN GZ_ADD_PLUGIN
#define EMSD_ADD_PLUGIN_ALIAS GZ_ADD_PLUGIN_ALIAS
#else
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/ExternalWorldWrenchCmd.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
namespace emsd
{
namespace gzsim = ignition::gazebo;
namespace gzmath = ignition::math;
namespace gzcommon = ignition::common;
namespace gzplugin = ignition::plugin;
}
#define EMSD_ADD_PLUGIN IGNITION_ADD_PLUGIN
#define EMSD_ADD_PLUGIN_ALIAS IGNITION_ADD_PLUGIN_ALIAS
#endif
