#ifndef THRUSTER_PLUGIN_H_
#define THRUSTER_PLUGIN_H_

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "maestro_emulator.h"
#include "thruster.h"

#include <iostream>
#include <vector>
#include <string>

using std::vector;

namespace gazebo
{
/**
 * ThrusterPlugin for Gazebo.
 */
class ThrusterPlugin : public ModelPlugin, public rclcpp::Node
{
    static constexpr std::array<char[20], 8> thruster_names =
                                                           {"dive_front_left",
                                                            "dive_front_right",
                                                            "dive_back_left",
                                                            "dive_back_right",
                                                            "strafe_front",
                                                            "strafe_back",
                                                            "forward_left",
                                                            "forward_right"};
public:
    ThrusterPlugin();
    ~ThrusterPlugin();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    virtual void Update();

private:
    // gazebo plugin objects
    event::ConnectionPtr updateConnection;

    // Thruster objects
    std::shared_ptr<MaestroEmulator> thruster_port;
    vector<Thruster> thrusters;
    double buoyancy_percentage;
    rclcpp::Duration visualizer_update_time;
    double surface_z;
    std::shared_ptr<rclcpp::SyncParametersClient> param_client;

    // gazebo messaging objects
    transport::PublisherPtr vis_pub;
    transport::NodePtr node;
    bool visualize_thrusters;
    rclcpp::Time last_update_time;

    void ReloadParams();
    void UpdateVisualizers();
    void UpdateBuoyancy();
    void UpdateThrusters();
};
}
#endif //THRUSTER_PLUGIN_H_
