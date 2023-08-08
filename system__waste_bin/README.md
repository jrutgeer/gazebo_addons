# Overview

This is a Gazebo Sim system plugin, that turns a model into a 'waste bin':
any model that makes contact with the waste bin model is deleted from the simulation.

A delay can be set so that a longer contact duration is needed before the contacting
model is removed.

See `include/waste_bin/WasteBin.hh` for more documentation.


# Build instructions

To build, open a terminal window and issue following commands:

```
  cd gazebo_addons
  colcon build --packages-select system_waste_bin
  source install/setup.bash
```


# Run the example world

After building and sourcing, run the example world by issuing following command:

```
  cd gazebo_addons
  gz sim system__waste_bin/worlds/waste_bin.sdf
```

# Acknowledgement

The code was inspired by the [ARIAC object disposal plugin](https://github.com/usnistgov/ARIAC/blob/ariac2023/ariac_plugins/include/ariac_plugins/object_disposal_plugin.hpp) for Gazebo classic, and based on the [touch plugin](https://github.com/gazebosim/gz-sim/tree/gz-sim7/src/systems/touch_plugin) and the [user commands](https://github.com/gazebosim/gz-sim/tree/gz-sim7/src/systems/user_commands) plugin from `gz-sim`.
