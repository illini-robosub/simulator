#include "marker_dropper.h"

namespace gazebo {

MarkerDropper::MarkerDropper() : Node("marker_dropper") {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
}

bool MarkerDropper::drop(
                   const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                         std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    physics::ModelPtr marker = world->ModelByName("marker");

    //Check if the marker is nullptr
    if(marker == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Marker pointer is Null");
        return false;
    }

    ignition::math::Pose3<double> sub_position = sub->WorldPose();

    RCLCPP_DEBUG(this->get_logger(), "Current Sub Position x: %f, y: %f, z: %f",
                 sub_position.Pos().X(), sub_position.Pos().Y(),
                 sub_position.Pos().Z());

    //This sets the marker spawn position relative to the sub
    sub_position.Pos().Z() -= 0.3;
    // This will be where the sub velocity is added to the marker
    //marker->SetLinearAccel(math::Vector3(0.5,0.5,0.5));
    marker->SetWorldPose(sub_position);

    RCLCPP_INFO(this->get_logger(), "Marker Dropped!");
    return true;
}

void MarkerDropper::Load(physics::ModelPtr _parent, sdf::ElementPtr) {
    sub = _parent;
    world = sub->GetWorld();

    std::string modelFilePath =
             ament_index_cpp::get_package_share_directory("robosub_simulator");
    modelFilePath += "/models/marker/marker.sdf";

    std::ifstream modelFile(modelFilePath);
    std::stringstream modelBuffer;
    modelBuffer << modelFile.rdbuf();

    world->InsertModelString(modelBuffer.str());

    service_server = this->create_service<std_srvs::srv::Empty>("drop_marker",
                                         std::bind(&MarkerDropper::drop, this,
                                                   std::placeholders::_1,
                                                   std::placeholders::_2));

    RCLCPP_INFO_STREAM(this->get_logger(),
                       service_server->get_service_name() << " started");

    RCLCPP_INFO(this->get_logger(), "Marker Dropper Plugin loaded");
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MarkerDropper)
}
