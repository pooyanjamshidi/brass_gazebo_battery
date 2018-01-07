
#ifndef BRASS_GAZEBO_BATTERY_BATTERY_DISCHARGE_H
#define BRASS_GAZEBO_BATTERY_BATTERY_DISCHARGE_H

#include <map>
#include <string>

#include "gazebo-7/gazebo/common/Plugin.hh"
#include "gazebo-7/gazebo/common/CommonTypes.hh"
#include "gazebo-7/gazebo/physics/physics.hh"

namespace gazebo
{
    /// \brief A plugin that simulate BRASS CP1 battery model: discharge and charge according to power models
    class BatteryPlugin : public ModelPlugin
    {

    };
}

#endif //BRASS_GAZEBO_BATTERY_BATTERY_DISCHARGE_H


