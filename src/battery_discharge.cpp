#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Battery.hh"
#include "gazebo/physics/physics.hh"
#include "battery_discharge.hh"
#include "gazebo/common/CommonTypes.hh"
#include <string>
#include <functional>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin);

BatteryPlugin::BatteryPlugin()
{
    this->c = 0.0;
    this->r = 0.0;
    this->tau = 0.0;

    this->e0 = 0.0;
    this->e1 = 0.0;

    this->q0 = 0.0;
    this->q = 0.0;

    this->iraw = 0.0;
    this->ismooth = 0.0;
}




