# Functionality
A Gazebo plugin that simulate an open-circuit battery model for challenge problem 1 of the BRASS Project.

# Support
This plugin is tested for ROS kinetic and Gazebo 7.8.1.

# Build
```$bash
cmake ../
make    
```

# Usage

In the brass.world file, `libbattery_discharge.so` is mentioned as a plugin. 
This implied that plugin is initialized and loaded when `p2-cp1.world` is opened in Gazebo. 
The xml code could be linked to any model in a new `.world` file.
```xml
<plugin name="battery" filename="libbattery_discharge.so">
    <ros_node>battery_monitor_client</ros_node>
    <link_name>body</link_name>
    <battery_name>linear_battery</battery_name>
    <constant_coef>12.694</constant_coef>
    <linear_coef>-3.1424</linear_coef>
    <initial_charge>1.1665</initial_charge>
    <capacity>1.2009</capacity>
    <resistance>0.061523</resistance>
    <smooth_current_tau>1.9499</smooth_current_tau>
    <charge_rate>0.2</charge_rate>
</plugin>
<plugin name="consumer" filename="libbattery_consumer.so">
    <link_name>body</link_name>
    <battery_name>linear_battery</battery_name>
    <power_load>6.6</power_load>
</plugin>
```

# Commands
TODO


