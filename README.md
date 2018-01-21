# Functionality
A Gazebo plugin that simulate an open-circuit battery model.

# Support
This plugin is tested for ROS kinetic and Gazebo 7.8.1.

# Build
```$bash
cmake ../
make    
```

# Usage

In the brass.world file, `libcontrol_light.so` is mentioned as a plugin. 
This implied that plugin is initialized and loaded when `brass.world` is opened in Gazebo. 
The xml code could be linked to any model in a new `.world` file.
```
<plugin name="contro_light" filename="libcontrol_light.so"/>
```

# Commands
TODO


