#include <rclcpp/rclcpp.hpp>

/*
 * A simple node that stores parameters and allows them to be accessed
 * by other nodes through a service.
 */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions opt;
    opt.allow_undeclared_parameters(true);
    opt.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<rclcpp::Node>("sim_param_server", opt);

    rclcpp::spin(node);

    return 0;
}
