add_library(thruster SHARED thruster_plugin.cpp thruster.cpp maestro_emulator.cpp)
ament_target_dependencies(thruster rclcpp std_msgs robosub_msgs robosub)
target_link_libraries(thruster ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} robosub::serial)
install(TARGETS thruster
        LIBRARY DESTINATION share/${PROJECT_NAME}/plugins)

add_library(buoyancy_improved SHARED buoyancy_improved.cpp)
ament_target_dependencies(buoyancy_improved rclcpp std_msgs)
target_link_libraries(buoyancy_improved ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS buoyancy_improved
        LIBRARY DESTINATION share/${PROJECT_NAME}/plugins)

# Plugin for the Marker Dropper
add_library(markerdrop SHARED marker_dropper.cpp)
ament_target_dependencies(markerdrop rclcpp std_msgs)
target_link_libraries(markerdrop ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS markerdrop
        LIBRARY DESTINATION share/${PROJECT_NAME}/plugins)

# Plugin for continually spinning the roulette wheel
#add_library(roulette_spinner SHARED roulette_spinner.cpp)
#ament_target_dependencies(roulette_spinner rclcpp std_msgs)
#target_link_libraries(roulette_spinner ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

# Plugin for the Torpedo Shooter
add_library(torpedoshoot SHARED torpedo_shooter.cpp)
ament_target_dependencies(torpedoshoot rclcpp std_msgs)
target_link_libraries(torpedoshoot ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS torpedoshoot
        LIBRARY DESTINATION share/${PROJECT_NAME}/plugins)

#simulator bridge
add_executable(simulator_bridge simulator_bridge.cpp)
ament_target_dependencies(simulator_bridge rclcpp std_msgs tf2 tf2_ros robosub_msgs sensor_msgs gazebo_msgs)
install(TARGETS simulator_bridge
        DESTINATION lib/${PROJECT_NAME})

# Localization engine diag harness/error calculator
add_executable(localization_harness localization_harness.cpp)
ament_target_dependencies(localization_harness rclcpp std_msgs tf2 tf2_ros robosub_msgs)

# parameter server
add_executable(parameter_server parameter_server.cpp)
ament_target_dependencies(parameter_server rclcpp)
install(TARGETS parameter_server
        DESTINATION lib/${PROJECT_NAME})

