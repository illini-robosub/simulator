#include "thruster_plugin.h"
#include <string>

using namespace std::literals::chrono_literals;

namespace gazebo {

constexpr std::array<char[20], 8> ThrusterPlugin::thruster_names;

ThrusterPlugin::ThrusterPlugin() : Node("thruster_plugin"),
                                   visualizer_update_time(0) {
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    thruster_port = std::make_shared<MaestroEmulator>();

    last_update_time = thruster_port->get_clock()->now();

    param_client = std::make_shared<rclcpp::SyncParametersClient>(this,
                                                           "/sim_param_server");
}

ThrusterPlugin::~ThrusterPlugin() { }

void ThrusterPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    RCLCPP_INFO(this->get_logger(), "Initializing thruster plugin.");

    // This sets gazebo msg publisher
    node = transport::NodePtr(new transport::Node());
    node->Init(_parent->GetWorld()->Name());
    vis_pub = node->Advertise<msgs::Visual>("~/visual", 80);

    while(!param_client->wait_for_service(2s)) {
        RCLCPP_ERROR(this->get_logger(),
                     "unable to query parameter server, retrying...");
    }

    RCLCPP_INFO(this->get_logger(), "connected to parameter server");

    while(!param_client->has_parameter("vsp_initialized")) {
        RCLCPP_INFO(this->get_logger(), "waiting for vsp to start");
        rclcpp::Rate(1).sleep();
    }

    auto params = param_client->get_parameters({"thrusters.max_thrust",
                                         "simulator.ports.simulated_thruster"});

    double max_thrust = 0;
    string thruster_virtual_port;
    bool thrust_loaded = false;
    bool port_loaded = false;

    for (auto & param : params) {
        if (param.get_name() == "thrusters.max_thrust") {
            max_thrust = param.as_double();
            thrust_loaded = true;
        }
        if (param.get_name() == "simulator.ports.simulated_thruster") {
            thruster_virtual_port = param.as_string();
            port_loaded = true;
        }
    }

    if (!thrust_loaded) {
        RCLCPP_FATAL(this->get_logger(),
                     "Failed to load maximum thrust value.");
        return;
    }

    if (!port_loaded) {
        RCLCPP_FATAL(this->get_logger(),
                     "Failed to load thruster simulated serial port.");
        return;
    }

    if (thruster_port->init(thruster_virtual_port)) {
        RCLCPP_FATAL(this->get_logger(),
                     "Failed to initialize emulated thrusters.");
        return;
    }

    // Get z coords of top of water
    physics::ModelPtr ceiling = _parent->GetWorld()->ModelByName(
                                                          "ceiling_plane");
    if (ceiling) {
        surface_z = ceiling->WorldPose().Pos().Z();
    } else {
        surface_z = 0.0;
        gzwarn <<
            "ceiling_plane model missing. assuming top of fluid is at z == 0"
            << std::endl;
    }

    for(std::string name : thruster_names) {
        thrusters.push_back(Thruster(name, _parent,
                                     max_thrust, *thruster_port, surface_z));
    }

    ReloadParams();

    // Set up Update to be called every simulation update
    // (which is frequently)
    updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ThrusterPlugin::Update, this));

    RCLCPP_INFO(this->get_logger(),
                "Thruster plugin successfully initialized.");
}

void ThrusterPlugin::ReloadParams() {
    if(!param_client->wait_for_service(2s)) {
        RCLCPP_WARN(this->get_logger(), "unable to query parameter server");
        return;
    }

    if(!param_client->has_parameter("simulator.visualize_thrusters")) {
        RCLCPP_WARN(this->get_logger(),
                    "failed to load thruster visualizer on/off");
    } else {
        visualize_thrusters = param_client->get_parameter(
                                       "simulator.visualize_thrusters", true);
    }

    double visualizer_update_rate = 10.0;
    if(!param_client->has_parameter("simulator.visualizer_update_rate")) {
        RCLCPP_WARN(this->get_logger(), "no visualizer update rate specified");
    } else {
        visualizer_update_rate = param_client->get_parameter(
                                     "simulator.visualizer_update_rate", 10.0);
    }
    visualizer_update_time = rclcpp::Duration::from_seconds(
                                               1.0 / visualizer_update_rate);
}

void ThrusterPlugin::Update() {
    // Update the thruster emulator to read all data out of the serial port.
    if (thruster_port->update()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to update virtual thruster emulator.");
        return;
    }

    /*
     * Add link forces to the frame from each thruster.
     */
    for(unsigned int i = 0; i < thrusters.size(); i++) {
        thrusters[i].addLinkForce();
    }

    // The reason this is being run at a constant rate now is because I ran
    // into a wierd issue where the visualizers would disappear (despite the
    // correct message being sent and received by gazebo) if the topic has not
    // been published on for some time. It's possible this is due to gazebo
    // auto disabling the visual entity but there is nothing in the visual msg
    // that would enable you to disable any auto disabling. Although messages
    // are still being sent even with visualizers off, I don't think this is in
    // dire need of optimization right now.
    if(thruster_port->get_clock()->now() - last_update_time >
                                           visualizer_update_time) {
        last_update_time = thruster_port->get_clock()->now();

        /*
         * Update the visualizers for each thruster.
         */
        for(unsigned int i = 0; i < thrusters.size(); i++) {
            msgs::Visual vis = thrusters[i].getVisualizationMessage();
            vis.set_visible(visualize_thrusters);
            vis_pub->Publish(vis);
        }
        ReloadParams();
    }
}

GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)
}
