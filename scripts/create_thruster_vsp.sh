#!/bin/bash

# Utilize socat to create a virtual serial port
socat pty,raw,echo=0,link=to_thruster_vsp pty,raw,echo=0,link=from_thruster_vsp &
socat_pid=$!

# Get the absolute paths of the ports.
port_to_thrusters="`pwd`/to_thruster_vsp"
port_from_thrusters="`pwd`/from_thruster_vsp"

# Wait for parameter server to start
until ros2 node info /sim_param_server > /dev/null 2>&1
do
    echo "waiting for parameter server to start"
    sleep 2
done

# Send port names to parameter server
until ros2 param set "/sim_param_server" "ports.thruster" "$port_to_thrusters"
do
    sleep 2
done
until ros2 param set "/sim_param_server" \
    "simulator.ports.simulated_thruster" "$port_from_thrusters"
do
    sleep 2
done
until ros2 param set "/sim_param_server" "vsp_initialized" "True"
do
    sleep 2
done
echo "vsp parameters set"

# Loop forever
while true
do
    sleep 60
done

trap "kill $socat_pid" EXIT
