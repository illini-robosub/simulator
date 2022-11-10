#ifndef MARKER_DROPPER_H_
#define MARKER_DROPPER_H_

#include <ignition/math/Pose3.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include <gazebo/transport/transport.hh>
#include "gazebo/gazebo.hh"

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include "std_srvs/srv/empty.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>

namespace gazebo
{
class MarkerDropper : public ModelPlugin, public rclcpp::Node
{
  private:
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_server;
    physics::ModelPtr sub;
    physics::WorldPtr world;

    bool drop(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
              std::shared_ptr<std_srvs::srv::Empty::Response> res);

  public:
    MarkerDropper();
    ~MarkerDropper() {}
    void Load(physics::ModelPtr _parent, sdf::ElementPtr);
};
}
#endif // MARKER_DROPPER_H_
