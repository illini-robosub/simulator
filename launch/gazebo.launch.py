import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    os.chdir(get_package_share_directory('robosub_simulator'))

    world_file = [get_package_share_directory('robosub_simulator'),
                  '/worlds/',
                  LaunchConfiguration('world')]

    cobalt_yaml = os.path.join(get_package_share_directory('robosub'),
                               'param', 'cobalt.yaml')
    simulator_yaml = os.path.join(
                               get_package_share_directory('robosub_simulator'),
                               'param', 'simulator.yaml')
    obstacles_yaml = os.path.join(
                               get_package_share_directory('robosub_simulator'),
                               'param', 'obstacles.yaml')
    cobalt_sim_yaml = os.path.join(
                               get_package_share_directory('robosub_simulator'),
                               'param', 'cobalt_sim.yaml')

    create_thruster_vsp = os.path.join(
                               get_package_share_directory('robosub_simulator'),
                               'scripts', 'create_thruster_vsp.sh')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='transdec.world',
                              description='name of the world file to load'),

        Node(package='robosub_simulator',
             executable='parameter_server',
             name='sim_param_server',
             parameters=[cobalt_yaml, cobalt_sim_yaml, simulator_yaml,
                         {'use_sim_time': True}]),

        ExecuteProcess(cmd=['bash', create_thruster_vsp], output='screen'),

        Node(package='robosub_simulator',
             executable='simulator_bridge',
             name='simulator_bridge',
             parameters=[simulator_yaml, obstacles_yaml, cobalt_sim_yaml,
                         {'use_sim_time': True}]),

        Node(package='robosub',
             executable='imu',
             name='imu',
             parameters=[{'active_imu': 'real'}, {'use_sim_time': True}]),

        Node(package='robosub',
             executable='thruster_maestro',
             name='thruster',
             parameters=[cobalt_yaml, cobalt_sim_yaml, {'use_sim_time': True}]),

        Node(package='robosub',
             executable='control',
             name='control',
             parameters=[cobalt_yaml, cobalt_sim_yaml, {'use_sim_time': True}]),

        Node(package='robosub',
             executable='pinger_bearing',
             name='pinger_bearing',
             parameters=[cobalt_yaml, cobalt_sim_yaml, {'use_sim_time': True}]),

        ExecuteProcess(cmd=['gzserver', world_file, '--verbose',
                            '-s', 'libgazebo_ros_init.so',
                            '-s', 'libgazebo_ros_factory.so',
                            '-s', 'libgazebo_ros_force_system.so'],
                       output='screen'),

        ExecuteProcess(cmd=['gzclient', '--verbose'], output='screen')

    ])
