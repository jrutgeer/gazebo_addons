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

After this, run the example by issuing following command:

```
  gz sim system__waste_bin/worlds/waste_bin.sdf
```



