# Gazebo Simulation

This directory contains the code for simulating the Viper quadcopter in Gazebo Fortress. This version was selected as development was on an Ubuntu 22.04 machine.

If you wish to launch the model directly from command line without ROS 2 bridge, then `cd` into `gazebo` directory and run:

```bash
ign gazebo worlds/world.sdf
```
This will run with the `sdf` files you see in the `gazebo` dev directory. If simulation launched with `viper-sim.py`, it's actually running from the `install/viper/share` directory. Make sure to `colcon build` to copy it over.

Then in another terminal, run the following to make drone fly:

```bash
ign topic -t /X3/gazebo/command/motor_speed --msgtype ignition.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
```

Reset pose:
```bash
ign service -s /world/quadcopter_teleop/set_pose \
    --reqtype ignition.msgs.Pose \
    --reptype ignition.msgs.Boolean \
    --req "name: 'viper', position: {x: 0, y: 0, z: 3}, orientation: {x: 0, y: 0, z: 0, w: 1}" \
    --timeout 2000
```

## PID Tuning
Within the `model.sdf` file, at the very bottom, we can change the anchor type to lock the quadopter along certain axis or in certain positions in space, allowing for easier PID tuning.

## Directory Structure
```bash
gazebo
├── config
│   └── bridge.yaml
├── env-hooks
│   └── viper.dsv.in  # Defines the IGN_GAZEBO_RESOURCE_PATH
├── models
│   └── viper
│       ├── model.config  # Needed for loading model file correctly using the models/viper directory
│       └── model.sdf
├── README.md
└── worlds
    └── world.sdf
```

`IGN_GAZEBO_RESOURCE_PATH` curently points to `gazebo` directory. This allows launching of Gazebo simulation independently by just calling `ign gazbeo worlds/world.sdf` from `gazebo` directory.


