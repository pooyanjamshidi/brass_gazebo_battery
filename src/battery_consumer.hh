#ifndef BRASS_GAZEBO_BATTERY_BATTERY_CONSUMER_H
#define BRASS_GAZEBO_BATTERY_BATTERY_CONSUMER_H

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"

namespace gazebo
{
    class GAZEBO_VISIBLE BatteryConsumerPlugin : public ModelPlugin
    {
    // Constructor
    public: BatteryConsumerPlugin();

    public: ~BatteryConsumerPlugin();

    // Inherited from ModelPlugin
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Battery
    private: common::BatteryPtr battery;

    // Consumer identifier
    private: int32_t consumerId;

    };

}

#endif //BRASS_GAZEBO_BATTERY_BATTERY_CONSUMER_H
