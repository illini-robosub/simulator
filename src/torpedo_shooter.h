#ifndef TORPEDO_SHOOTER_H_
#define TORPEDO_SHOOTER_H_

#include <ignition/math/Pose3.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include <gazebo/transport/transport.hh>
#include "gazebo/gazebo.hh"

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include "std_srvs/srv/trigger.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>

namespace gazebo
{
class TorpedoShooter : public ModelPlugin, public rclcpp::Node
{
  private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_server;
    physics::ModelPtr sub;
    physics::WorldPtr world;

    bool shoot(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  public:
    TorpedoShooter();
    ~TorpedoShooter() {}
    void Load(physics::ModelPtr _parent, sdf::ElementPtr);
};
}
#endif // TORPEDO_SHOOTER_H_
