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
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // E(t) = e0 + e1* Q(t)/c
    protected: double e0;
    protected: double e1;

    // Initial battery charge in Ah.
    protected: double q0;

    // Battery capacity in Ah.
    protected: double c;

    // Battery inner resistance in Ohm
    protected: double r;

    // Current low-pass filter characteristic time in seconds.
    protected: double tau;

    // Raw battery current in A.
    protected: double iraw;

    // Smoothed battery current in A.
    protected: double ismooth;

    // Instantaneous battery charge in Ah.
    protected: double q;



    };
}

#endif //BRASS_GAZEBO_BATTERY_BATTERY_DISCHARGE_H


