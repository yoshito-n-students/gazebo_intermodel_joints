# gazebo_intermodel_joints
A gazebo plugin which spawns joints between 2 models

## Quick usage
If you want to build a robot consisting of a mobile base and two manipulators which have been independently developed, you can compose it for Gazebo simulation by using this plugin.
```xml
<!-- In your .world file -->

<!-- Ground vehicle as a mobile base -->
<include>
    <uri>model://vehicle</uri>
    <name>mobile_base</name>
</include>

<!-- Manipulator as a left arm -->
<include>
    <uri>model://manipulator</uri>
    <name>left_arm</name>
</include>

<!-- Another manipulator as a right arm -->
<include>
    <uri>model://manipulator</uri>
    <name>right_arm</name>
</include>

<!-- Couplers between the mobile base and arms -->
<puligin name="couplers" filename="libIntermodelJoints.so">
    <!-- Intermodel joints definition using regular sdf format -->
    <joint name="mobile_base::left_coupler" type="fixed">
        <parent>mobile_base::body</parent>
        <child>left_arm::base</child>
    </joint>
    <joint name="mobile_base::right_coupler" type="fixed">
        <parent>mobile_base::body</parent>
        <child>right_arm::base</child>
    </joint>
</plugin>
```
or the final block can be a additional model as follows.
```xml
<!-- Couplers between the mobile base and arms -->
<model name="couplers" />
<puligin name="coupler_joints" filename="libIntermodelJoints.so">
    <!-- Intermodel joints definition using regular sdf format -->
    <joint name="couplers::left_coupler" type="fixed">
        <parent>mobile_base::body</parent>
        <child>left_arm::base</child>
    </joint>
    <joint name="couplers::right_coupler" type="fixed">
        <parent>mobile_base::body</parent>
        <child>right_arm::base</child>
    </joint>
</plugin>
```

## Other specifications
* Tested with Gazebo7 with ROS Kinetic & Gezebo9 with ROS Melodic
* Supports all joint types
* Supports any number of joints

## Why is this plugin required?
**Nested model does not work**  
The following defition is valid but does not work as you expected because [gazebo_ros_control plugin](http://wiki.ros.org/gazebo_ros_control) does not support nested models.
```xml
<model name="mobile_manipulator">
    <include>
        <!-- gazebo_ros_control plugin does not support nested model! -->
        <uri>model://vehicle</uri>
    </include>
    <include>
        <!-- gazebo_ros_control plugin does not support nested model! -->
        <uri>model://manipulator</uri>
    </include>

    <joint name="coupler" type="fixed">
        <parent>vehicle::body</parent>
        <child>manipulator::base</child>
    </joint>
</model>
```
**External model does not work**  
Also, the following definition is invalid because Gazebo usually searches the parent link from the joint's parent model (the only exception is the "world" link).
```xml
<include>
    <uri>model://vehicle</uri>
</include>
<include>
    <uri>model://manipulator</uri>
</include>

<model name="coupler_model">
    <joint name="coupler" type="fixed">
        <!-- Parent link must belong to coupler_model or be "world" -->
        <parent>vehicle::body</parent>
        <child>manipulator::base</child>
    </joint>
</model>
```