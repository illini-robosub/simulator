#include "torpedo_shooter.h"

namespace gazebo {

TorpedoShooter::TorpedoShooter() : Node("torpedo_shooter") {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
}

bool TorpedoShooter::shoot(
                     const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    // Need to make a torpedo model
    physics::ModelPtr torpedo = world->ModelByName("torpedo");

    //Check if the torpedo model is nullptr
    if(torpedo == nullptr) {
        RCLCPP_ERROR(this->get_logger(), "Torpedo pointer is Null");
        return false;
    }

    ignition::math::Pose3<double> sub_position = sub->WorldPose();

    RCLCPP_INFO(this->get_logger(), "Current Sub Position x: %f, y: %f, z: %f",
                sub_position.Pos().X(), sub_position.Pos().Y(),
                sub_position.Pos().Z());

    RCLCPP_INFO(this->get_logger(),
                "Current Sub rotation roll: %f, pitch: %f, yaw: %f",
                sub_position.Rot().Euler().X(), sub_position.Rot().Euler().Y(),
                sub_position.Rot().Euler().Z());

    //This sets the torpedo spawn position relative to the sub
    sub_position.Pos().Z() -= 0.4;
    //sub_position.pos.y += 1;
    // This will be where the sub velocity is added to the marker
    // torpedo->SetLinearAccel(math::Vector3(0.0,0.0,.001));


    // This is for calculating the direction of the initial velocity of
    // the torpedo
    // Using just trig
    auto magnitude =
                  ignition::math::Vector3(cos(sub_position.Rot().Euler().Z()),
                      sin(sub_position.Rot().Euler().Z()), 0.0);

    torpedo->SetLinearVel(magnitude);
    torpedo->SetWorldPose(sub_position);

    RCLCPP_INFO(this->get_logger(), "Torpedo Fired!");
    res->success = true;
    res->message = "Torpedoes Away!";
    return true;
}

void TorpedoShooter::Load(physics::ModelPtr _parent, sdf::ElementPtr) {
    sub = _parent;
    world = sub->GetWorld();

    std::string modelFilePath =
             ament_index_cpp::get_package_share_directory("robosub_simulator");
    modelFilePath += "/models/torpedo/torpedo.sdf";

    std::ifstream modelFile(modelFilePath);
    std::stringstream modelBuffer;
    modelBuffer << modelFile.rdbuf();

    world->InsertModelString(modelBuffer.str());

    service_server = this->create_service<std_srvs::srv::Trigger>(
                                          "shoot_torpedo",
                                          std::bind(&TorpedoShooter::shoot,
                                          this, std::placeholders::_1,
                                          std::placeholders::_2));

    RCLCPP_INFO_STREAM(this->get_logger(),
                       service_server->get_service_name() << " started");

    RCLCPP_INFO(this->get_logger(), "Torpedo Shooter Plugin loaded");
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TorpedoShooter)
}
