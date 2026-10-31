# ARVA Simulator
This ROS2 node is a simple implementaion to simulate am ARVA sensor. The working princople is the following:
- Places a transmitter randomly inside the set searchfild.
- computes the expected fluxline distance and the offset angle to the fluxline tangent based on the drones positon and heading
- simulates the msgs as if the system has an ARVA sensor pluges in

Configure the transmitter in `cfg/arva_sim.yaml`:

```yaml
arva_sim_node:
  ros__parameters:
    transmitter_position_ned_m: [30.0, 20.0, 0.0]
    transmitter_heading_deg: -90.0
    receiver_vertical_offset_m: 0.65
    angle_valid_min_distance_m: 4.0
    max_detection_range_m: 15.0
```

Position uses PX4 local NED coordinates in meters: North, East, Down. Heading is
in degrees and retains the simulator's existing dipole rotation convention.
`receiver_vertical_offset_m` is subtracted from the rounded PX4 Down coordinate,
preserving the previous coordinate conversion. `angle_valid_min_distance_m`
sets the receiver-distance threshold below which (and at which) the angle is
invalid; with a valid distance this triggers the executor's fine-search handoff.
`max_detection_range_m` limits the receiver distance in the far-field signal
calculation, rather than the geometric transmitter-to-drone distance. These
three distances are in meters and retain their previous defaults.

All parameters are read at startup. After editing the YAML, rebuild `arva_sim`,
source the workspace, and restart the simulator. The standard simulator and
ARVA master launch files load this configuration automatically.

```sh
colcon build --packages-select arva_sim
source install/setup.bash
ros2 launch arva_sim arva_sim.launch.py
```

When running the node directly, pass the parameter file explicitly:

```sh
ros2 run arva_sim arva_sim_node --ros-args --params-file \
  src/roscon-25-workshop/px4_roscon_25/arva_sim/cfg/arva_sim.yaml
```
