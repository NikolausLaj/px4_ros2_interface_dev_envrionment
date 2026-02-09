# ARVA Simulator
This ROS2 node is a simple implementaion to simulate am ARVA sensor. The working princople is the following:
- Places a transmitter randomly inside the set searchfild.
- computes the expected fluxline distance and the offset angle to the fluxline tangent based on the drones positon and heading
- simulates the msgs as if the system has an ARVA sensor pluges in
