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
#include <kobuki_msgs/MotorPower.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

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

    #ifdef BATTERY_DEBUG
        gzdbg << "Constructed BatteryPlugin and initialized parameters. \n";
    #endif
    ROS_INFO_STREAM("BRASS CP1 battery is loaded.");
}

BatteryPlugin::~BatteryPlugin()
{
#ifdef BATTERY_DEBUG
    gzdbg << "Destructing BatteryPlugin and removing the ros node. \n";
#endif
    this->rosNode->shutdown();
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

    this->sim_time_now = this->world->GetSimTime().Double();

    #ifdef BATTERY_DEBUG
        gzdbg << "Loading the BatteryPlugin at time:" << this->sim_time_now << "\n";
    #endif


    // Create ros node and publish stuff there!
    this->rosNode.reset(new ros::NodeHandle(_sdf->Get<std::string>("battery_discharge_client")));
    this->motor_power = this->rosNode->advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
    this->charge_state = this->rosNode->advertise<std_msgs::Float64>(this->world->GetName() + "/charge_level", 1);

    this->set_charging = this->rosNode->advertiseService(this->model->GetName() + "/set_charging", &BatteryPlugin::SetCharging, this);
    this->set_charge = this->rosNode->advertiseService(this->model->GetName() + "/set_charge", &BatteryPlugin::SetCharge, this);
    this->set_coefficients = this->rosNode->advertiseService(this->model->GetName() + "/set_model_coefficients", &BatteryPlugin::SetModelCoefficients, this);

    std::string linkName = _sdf->Get<std::string>("link_name");
    this->link = this->model->GetLink(linkName);

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

    this->sim_time_now = this->world->GetSimTime().Double();

    #ifdef BATTERY_DEBUG
        gzdbg << "Loaded the BatteryPlugin at time:" << this->sim_time_now << "\n";
    #endif
}

void BatteryPlugin::Init()
{
    this->q = this->q0;
    this->charging = false;
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

    this->sim_time_now = this->world->GetSimTime().Double();

    #ifdef BATTERY_DEBUG
        gzdbg << "Current charge:" << this->q << "at:" << this->sim_time_now << "\n";
    #endif

    this->et = this->e0 + this->e1 * (1 - this->q / this->c) - this->r * this->ismooth;

    #ifdef BATTERY_DEBUG
        gzdbg << "Current voltage:" << this->et << "at:" << this->sim_time_now << "\n";
    #endif

    //Turn off motor
    if (this->q <= 0)
    {
        this->sim_time_now = this->world->GetSimTime().Double();

        #ifdef BATTERY_DEBUG
            gzdbg << "Out of juice at:" << this->sim_time_now << "\n";
        #endif

        this->q = 0;
        kobuki_msgs::MotorPower power_msg;
        power_msg.state = 0;
        lock.lock();
        this->motor_power.publish(power_msg);
        lock.unlock();
    }

    std_msgs::Float64 charge_msg;
    charge_msg.data = this->q;
    lock.lock();
    this->charge_state.publish(charge_msg);
    lock.unlock();

    return et;

}

bool BatteryPlugin::SetCharging(brass_gazebo_battery::SetCharging::Request& req,
                                brass_gazebo_battery::SetCharging::Response& res)
{

    lock.lock();
    this->charging = req.charging;
    if (this->charging) {
#ifdef BATTERY_DEBUG
        gzdbg << "Bot is charging" << "\n";
#endif
    }
    else{
#ifdef BATTERY_DEBUG
        gzdbg << "Bot disconnected from the charging station" << "\n";
#endif
    }
    lock.unlock();
    res.result = true;
    return true;
}

bool BatteryPlugin::SetCharge(brass_gazebo_battery::SetCharge::Request &req,
                              brass_gazebo_battery::SetCharge::Response &res)
{
    lock.lock();
    this->q = req.charge;
    #ifdef BATTERY_DEBUG
        gzdbg << "Received charge:" << this->q << "\n";
    #endif
    lock.unlock();
    res.result = true;
    return true;
}

bool BatteryPlugin::SetModelCoefficients(brass_gazebo_battery::SetCoef::Request &req,
                                         brass_gazebo_battery::SetCoef::Response &res)
{
    lock.lock();
    this->e0 = req.constant_coef;
    this->e1 = req.linear_coef;
    #ifdef BATTERY_DEBUG
        gzdbg << "Power model is changed, new coefficients (constant, linear):" << this->e0 << this->e1 << "\n";
    #endif
    lock.unlock();
    res.result = true;
    return true;
}