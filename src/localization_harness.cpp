#include "rclcpp/rclcpp.hpp"
#include "robosub_msgs/msg/float32_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "tf2/transform_datatypes.h"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <string>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("localization_harness");

    rclcpp::Rate publishRate(30.0);

    tf2_ros::Buffer tfb(node->get_clock());
    tf2_ros::TransformListener tflr(tfb);

    geometry_msgs::msg::TransformStamped resultantTransform;

    auto linear_error_pub = node->create_publisher
                                  <robosub_msgs::msg::Float32Stamped>
                                  ("localization/error/linear", 1);

    auto vector_error_pub = node->create_publisher
                                  <geometry_msgs::msg::Vector3Stamped>
                                  ("localization/error/vector", 1);


    // Wait for a transform to be available between the localization engine's
    // position and the simulator's position.

    RCLCPP_DEBUG(node->get_logger(),
        "Waiting for available transformation from cobalt to cobalt_sim...");

    while (rclcpp::ok()) {
        try {
            resultantTransform = tfb.lookupTransform("cobalt_sim", "cobalt",
                                       rclcpp::Time(0),
                                       rclcpp::Duration::from_seconds(300));

            robosub_msgs::msg::Float32Stamped linear_error_msg;
            geometry_msgs::msg::Vector3Stamped vector_error_msg;

            linear_error_msg.header.stamp = vector_error_msg.header.stamp =
                node->get_clock()->now();

            vector_error_msg.vector = resultantTransform.transform.translation;
            linear_error_msg.data = tf2::Vector3(vector_error_msg.vector.x,
                                                 vector_error_msg.vector.y,
                                                 vector_error_msg.vector.z)
                                                 .length();

            linear_error_pub->publish(linear_error_msg);
            vector_error_pub->publish(vector_error_msg);
        }
        catch (tf2::LookupException const & ex) {
            RCLCPP_WARN(node->get_logger(),
                        "Caught LookupException: %s", ex.what());
        }

        publishRate.sleep();
    }

    return 0;
}
