# Gazebo Simulation

This directory contains the code for simulating the Viper quadcopter in Gazebo Fortress. This version was selected as development was on an Ubuntu 22.04 machine.

If you wish to launch the model directly from command line without ROS 2 bridge, then `cd` into `gazebo` directory and run:

```bash
ign gazebo worlds/world.sdf
```

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

## Directory Structure
```bash
gazebo/
├── config
│   └── bridge.yaml
├── models
│   └── viper
│       ├── model.config  # Needed for loading viper.sdf
│       └── viper.sdf
├── README.md
└── worlds
    └── world.sdf
```

This will allow Gazebo to find the correct model file.


