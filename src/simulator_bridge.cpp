#include "rclcpp/rclcpp.hpp"
#include "robosub_msgs/msg/float32_stamped.hpp"
#include "robosub_msgs/msg/euler.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/msg/model_state.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "robosub_msgs/msg/obstacle_pos_array.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "robosub_msgs/msg/hydrophone_deltas.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "robosub/utility/ThrottledPublisher.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

#include <cmath>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using std::vector;
using std::string;
using namespace Eigen;
using namespace rs;

static constexpr double _180_OVER_PI = 180.0 / 3.14159;

std::shared_ptr<rclcpp::Node> node;

ThrottledPublisher<geometry_msgs::msg::PointStamped> position_pub;
ThrottledPublisher<geometry_msgs::msg::QuaternionStamped> orientation_pub;
ThrottledPublisher<robosub_msgs::msg::Euler> euler_pub;
ThrottledPublisher<robosub_msgs::msg::Float32Stamped> depth_pub;
ThrottledPublisher<robosub_msgs::msg::ObstaclePosArray> obstacle_pos_pub;
ThrottledPublisher<robosub_msgs::msg::HydrophoneDeltas> hydrophone_deltas_pub;
ThrottledPublisher<geometry_msgs::msg::Vector3Stamped> lin_accel_pub;

// List of names of objects to publish the position and name of. This will be
// loaded from parameters.
std::vector<std::string> object_names;

Vector3d pinger_position;
Vector3d ceiling_plane_position;

// Helper function to publish the sub's position onto the TF tree
void publishTfFrame(const geometry_msgs::msg::Pose pose, string frame_id,
    string parent_frame_id = "world") {

    static tf2_ros::TransformBroadcaster tfbr(node);

    geometry_msgs::msg::TransformStamped transform;
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation = pose.orientation;
    transform.child_frame_id = frame_id;
    transform.header.stamp = node->get_clock()->now();
    transform.header.frame_id = parent_frame_id;

    tfbr.sendTransform(transform);
}

void linkStatesCallback(const gazebo_msgs::msg::LinkStates::ConstSharedPtr msg)
{
    int hydrophone_indices[4] = {-1, -1, -1, -1};
    for (unsigned int i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == "robosub::hydrophone_h0") hydrophone_indices[0] = i;
        if (msg->name[i] == "robosub::hydrophone_hx") hydrophone_indices[1] = i;
        if (msg->name[i] == "robosub::hydrophone_hy") hydrophone_indices[2] = i;
        if (msg->name[i] == "robosub::hydrophone_hz") hydrophone_indices[3] = i;
    }

    /*
     * Save the hydrophone positions into a vector. They are stored in
     * order:
     *      reference, x-axis, y-axis, z-axis
     */
    vector<Vector3d> hydrophone_positions;
    for (int i = 0; i < 4; ++i) {
        if (hydrophone_indices[i] == -1) {
            RCLCPP_ERROR_STREAM(node->get_logger(),
                                "Failed to find hydrophone " << i << " pose.");
            return;
        }

        hydrophone_positions.push_back(
                Vector3d(msg->pose[hydrophone_indices[i]].position.x,
                msg->pose[hydrophone_indices[i]].position.y,
                -(ceiling_plane_position[2] -
                    msg->pose[hydrophone_indices[i]].position.z)));
    }

    /*
     * Now that hydrophone positions are known, find the
     * distance of each hydrophone from the pinger and translate the
     * distance into signal time-of-flight as the ping travels through
     * the water.
     */
    vector<double> hydrophone_time_delays;

    for (unsigned int i = 0; i < hydrophone_positions.size(); ++i) {
        Vector3d delta = hydrophone_positions[i] - pinger_position;
        double distance = sqrt(delta[0]*delta[0] + delta[1] * delta[1]
                + delta[2] * delta[2]);

        /*
         * Calculate a ping signal time delay from the distance by
         * dividing by the speed of sound in water (1484 m/s).
         */
        double time_delay = distance / 1484.0;
        hydrophone_time_delays.push_back(time_delay);
    }

    /*
     * Subtract the reference time delay from each hyodrophone time
     * delay to calculate the time differences in receiving the
     * hydrophone signal on all of the ordinal hydrophones.
     */
    for (unsigned int i = 1; i < hydrophone_time_delays.size(); ++i) {
        hydrophone_time_delays[i] = hydrophone_time_delays[0] -
            hydrophone_time_delays[i];
    }

    robosub_msgs::msg::HydrophoneDeltas deltas;
    deltas.header.stamp = node->get_clock()->now();
    deltas.x_delta = rclcpp::Duration::from_seconds(hydrophone_time_delays[1]);
    deltas.y_delta = rclcpp::Duration::from_seconds(hydrophone_time_delays[2]);
    deltas.z_delta = rclcpp::Duration::from_seconds(hydrophone_time_delays[3]);

    hydrophone_deltas_pub.publish(deltas);
}

// ModelStates msg consists of a name, a pose (position and orientation), and a
// twist (linear and angular velocity) for each object in the simulator
// Currently it publishes the position, orientation, and depth of the sub, as
// well as a list (defined in params) of objects
void modelStatesCallback(const
                         gazebo_msgs::msg::ModelStates::ConstSharedPtr msg) {
    geometry_msgs::msg::PointStamped position_msg;
    geometry_msgs::msg::QuaternionStamped orientation_msg;
    robosub_msgs::msg::Float32Stamped depth_msg;
    robosub_msgs::msg::Euler euler_msg;

    // Find top of water and subs indices in modelstates lists
    int sub_index = -1, pinger_index = -1, ceiling_index = -1;
    for(unsigned int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robosub") sub_index = i;
        if (msg->name[i] == "ceiling_plane") ceiling_index = i;
        if (msg->name[i] == "pinger_a") pinger_index = i;
    }

    /*
     * Validate that all model states were successfully found.
     */
    if (sub_index == -1) {
        RCLCPP_ERROR_STREAM(node->get_logger(),
                            "Failed to locate submarine model state.");
        return;
    }

    if (pinger_index == -1) {
        RCLCPP_ERROR_STREAM(node->get_logger(),
                            "Failed to locate pinger model state.");
        return;
    }

    if (ceiling_index == -1) {
        RCLCPP_ERROR_STREAM(node->get_logger(),
                            "Failed to locate ceiling model state.");
        return;
    }

    /*
     * Update the global water top position.
     */
    ceiling_plane_position = Vector3d(msg->pose[ceiling_index].position.x,
            msg->pose[ceiling_index].position.y,
            msg->pose[ceiling_index].position.z);

    /*
     * Update the global pinger position. Depth of pinger is relative to water
     * top position
     */
    pinger_position = Vector3d(msg->pose[pinger_index].position.x,
            msg->pose[pinger_index].position.y,
            -(ceiling_plane_position[2] -
                msg->pose[pinger_index].position.z));

    RCLCPP_DEBUG_STREAM(node->get_logger(),
                        "pinger_position: " << pinger_position);

    // Calculate depth from the z positions of the water top and the sub
    depth_msg.data = -(msg->pose[ceiling_index].position.z -
                     msg->pose[sub_index].position.z);
    depth_msg.header.stamp = node->get_clock()->now();

    // Construct header for the sub's position
    position_msg.header.stamp = node->get_clock()->now();
    position_msg.header.frame_id = "world";

    // Copy sub pos to position msg
    position_msg.point.x = msg->pose[sub_index].position.x;
    position_msg.point.y = msg->pose[sub_index].position.y;
    position_msg.point.z = depth_msg.data;
    position_msg.point.x -= pinger_position[0];
    position_msg.point.y -= pinger_position[1];

    orientation_msg.quaternion.x = msg->pose[sub_index].orientation.x;
    orientation_msg.quaternion.y = msg->pose[sub_index].orientation.y;
    orientation_msg.quaternion.z = msg->pose[sub_index].orientation.z;
    orientation_msg.quaternion.w = msg->pose[sub_index].orientation.w;
    orientation_msg.header.stamp = node->get_clock()->now();

    tf2::Matrix3x3 m(tf2::Quaternion(orientation_msg.quaternion.x,
                                   orientation_msg.quaternion.y,
                                   orientation_msg.quaternion.z,
                                   orientation_msg.quaternion.w));

    m.getRPY(euler_msg.roll, euler_msg.pitch, euler_msg.yaw);

    /*
     * The actual BNO mounting position causes it to give positive roll as
     * rolling left and positive pitch as pitching up. As such, emulate the
     * sensor to also define values in this way.
     */
    tf2::Quaternion q;
    q.setRPY(euler_msg.roll * -1, euler_msg.pitch * -1, euler_msg.yaw);

    euler_msg.roll *= _180_OVER_PI;
    euler_msg.pitch *= _180_OVER_PI;
    euler_msg.yaw *= _180_OVER_PI;

    // Publish sub position and orientation

    geometry_msgs::msg::Pose subTFPose;
    subTFPose.orientation = orientation_msg.quaternion;
    subTFPose.position = position_msg.point;
    publishTfFrame(subTFPose, "cobalt");
    position_pub.publish(position_msg);
    orientation_pub.publish(orientation_msg);
    depth_pub.publish(depth_msg);
    euler_pub.publish(euler_msg);

    // Iterate through object_names and for each iteration search through the
    // msg.name array and attempt to find object_names[i]. If it is not found
    // std::find returns an iterator equal to msg.name.end(). If it is found
    // std::find returns an iterator pointing to the object with
    // object_name[i]. std::distance is used to find the index of that object
    // wthin the msg.name (and therefore msg.pose array since they contain the
    // same objects in the same order).
    robosub_msgs::msg::ObstaclePosArray object_array;
    for(unsigned int i = 0; i < object_names.size(); i++) {
        auto it = std::find(msg->name.begin(), msg->name.end(),
                            object_names[i]);

        // object_name[i] found in msg.name.
        if(it != msg->name.end()) {
            // Calculate index of object in msg.name from the iterator element.
            int idx = std::distance(msg->name.begin(), it);

            // Extract necessary data from modelstates.
            robosub_msgs::msg::ObstaclePos pos;
            pos.x = msg->pose[idx].position.x - pinger_position[0];
            pos.y = msg->pose[idx].position.y - pinger_position[1];
            pos.z = -(msg->pose[ceiling_index].position.z -
                    msg->pose[idx].position.z);

            pos.name = msg->name[idx];

            object_array.data.push_back(pos);
        }
    }

    obstacle_pos_pub.publish(object_array);
}

void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    geometry_msgs::msg::Vector3Stamped lin_accel;

    lin_accel.vector.x = msg->linear_acceleration.x;
    lin_accel.vector.y = msg->linear_acceleration.y;
    lin_accel.vector.z = msg->linear_acceleration.z;

    lin_accel.header.stamp = node->get_clock()->now();
    lin_accel_pub.publish(lin_accel);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("simulator_bridge");

    position_pub = ThrottledPublisher<geometry_msgs::msg::PointStamped>
        ("simulator/cobalt/position", 1, 0, node, "rate.simulator.position");
    orientation_pub = ThrottledPublisher<geometry_msgs::msg::QuaternionStamped>
        ("real/orientation", 1, 0, node, "rate.imu");
    euler_pub = ThrottledPublisher<robosub_msgs::msg::Euler>
        ("real/pretty/orientation", 1, 0, node, "rate.simulator.euler");
    depth_pub = ThrottledPublisher<robosub_msgs::msg::Float32Stamped>
        ("depth", 1, 0, node, "rate.depth");
    obstacle_pos_pub = ThrottledPublisher<robosub_msgs::msg::ObstaclePosArray>
        ("obstacles/positions", 1, 0, node, "rate.simulator.obstacle_pos");
    hydrophone_deltas_pub = ThrottledPublisher
                                        <robosub_msgs::msg::HydrophoneDeltas>
        ("hydrophones/delta", 1, 0, node,
         "rate.simulator.hydrophone_deltas");
    lin_accel_pub = ThrottledPublisher<geometry_msgs::msg::Vector3Stamped>
        ("real/acceleration/linear", 1, 0, node, "rate.simulator.lin_accel");

    auto orient_sub = node->create_subscription<gazebo_msgs::msg::ModelStates>
                                               ("model_states", 1,
                                                modelStatesCallback);

    auto link_sub = node->create_subscription<gazebo_msgs::msg::LinkStates>
                                             ("link_states", 1,
                                              linkStatesCallback);

    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>
                                            ("gazebo/sim_imu/out", 1,
                                             imuCallback);

    node->declare_parameter("rate.simulator.simulator_bridge");
    node->declare_parameter("obstacles");
    double rate;
    if(!node->get_parameter("rate.simulator.simulator_bridge", rate)) {
        RCLCPP_ERROR_STREAM(node->get_logger(),
                            "failed to load max simulator bridge rate");
        return 0;
    }
    rclcpp::Rate r(rate);

    // Put object names in vector.
    if(!node->get_parameter("obstacles", object_names)) {
        RCLCPP_WARN_STREAM(node->get_logger(), "failed to load obstacle names");
    }

    // I use spinOnce and sleeps here because the simulator publishes
    // at a very high rate and we don't need every message.
    // You can throttle subscriber input in other ways but this method doesn't
    // require extra packages or anything like that.
    while(rclcpp::ok()) {
        rclcpp::spin_some(node);
        r.sleep();
    }

    return 0;
}
