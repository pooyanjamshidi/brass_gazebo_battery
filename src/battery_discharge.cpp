#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Battery.hh"
#include "gazebo/physics/physics.hh"
#include "battery_discharge.hh"
#include "ros/ros.h"
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

void BatteryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // check if the ros is up!
    if (! ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "battery_discharge_client", ros::init_options::NoSigintHandler);
    }

    this->model = _model;
    this->world = _model->GetWorld();

    this->e0 = _sdf->Get<double>("constant_coef");
    this->e1 = _sdf->Get<double>("linear_coef");
    this->c = _sdf->Get<double>("capacity");
    this->r = _sdf->Get<double>("resistance");
    this->tau = _sdf->Get<double>("smooth_current_tau");
    std::string batteryName = _sdf->Get<std::string>("battery_name");
    // Creates the battery
    this->battery = this->link->Battery(batteryName);
    // Specifying a custom update function
    this->battery->SetUpdateFunc(std::bind(&BatteryPlugin::OnUpdateVoltage, this, std::placeholders::_1));
}

void BatteryPlugin::Init()
{
    this->q = this->q0;
}

void BatteryPlugin::Reset()
{
    this->iraw = 0.0;
    this->ismooth = 0.0;
    this->Init();
}

double BatteryPlugin::OnUpdateVoltage(const common::BatteryPtr &_battery)
{
    double dt = this->world->GetPhysicsEngine()->GetMaxStepSize();
    double totalpower = 0.0;
    double k = dt / this->tau;

    if (fabs(_battery->Voltage())<1e-3)
        return 0.0;

    for (auto powerLoad : _battery->PowerLoads())
        totalpower += powerLoad.second;

    this->iraw = totalpower / _battery->Voltage();

    this->ismooth = this->ismooth + k * (this->iraw - this->ismooth);

    this->q = this->q - GZ_SEC_TO_HOUR(dt * this->ismooth);

    return this->e0 + this->e1 * (1 - this->q / this->c) - this->r * this->ismooth;

}