
#ifndef BRASS_GAZEBO_BATTERY_BATTERY_DISCHARGE_H
#define BRASS_GAZEBO_BATTERY_BATTERY_DISCHARGE_H

#include <map>
#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
    /// \brief A plugin that simulate BRASS CP1 battery model: discharge and charge according to power models
    class BatteryPlugin : public ModelPlugin
    {
        /// \brief Constructor
    public: BatteryPlugin();

        // Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::Elementptr _sdf);




    };
}

#endif //BRASS_GAZEBO_BATTERY_BATTERY_DISCHARGE_H


