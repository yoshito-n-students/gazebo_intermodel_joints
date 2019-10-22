# gazebo_intermodel_joints
A gazebo plugin which spawns joints between two models

## Quick usage
For example, if you want to build a robot consisting of a mobile base and two manipulators which have been independently developed, you can define it for Gazebo simulation by using this plugin. In the following example, the pulugin creates coupler joints in the mobile base model.
```xml
<!-- In your .world file -->
<world name="world">
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
    <pulgin name="couplers" filename="libIntermodelJoints.so">
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
</world>
```
or you can create the coupler joints in another model as follows.
```xml
<!-- Couplers between the mobile base and arms -->
<model name="couplers" />
<pulgin name="coupler_joints" filename="libIntermodelJoints.so">
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
* Tested with Gazebo 7 with ROS Kinetic on Ubuntu 16.04 & Gezebo 9 with ROS Melodic on Ubuntu 18.04
* Supports all joint types
* Supports any number of joints

## Why is this plugin required?
**Reason 1: Nested model does not work**  
The following definition does not work with ros_control because [gazebo_ros_control plugin](http://wiki.ros.org/gazebo_ros_control) does not support this use case.
```xml
<model name="mobile_manipulator">
    <include>
        <!-- NG: gazebo_ros_control plugin in the vehicle does not work -->
        <uri>model://vehicle</uri>
    </include>
    <include>
        <!-- NG: gazebo_ros_control plugin in the manipulator does not work -->
        <uri>model://manipulator</uri>
    </include>

    <joint name="coupler" type="fixed">
        <!-- OK: Gazebo can find links -->
        <parent>vehicle::body</parent>
        <child>manipulator::base</child>
    </joint>
</model>
```
**Reason 2: External model basically does not work**  
Also, the following definition does not work because Gazebo normally searches the parent link from the joint's parent model (the only exception is the "world" link).
```xml
<include>
    <!-- OK: gazebo_ros_control plugin works -->
    <uri>model://vehicle</uri>
</include>
<include>
    <!-- OK: gazebo_ros_control plugin works -->
    <uri>model://manipulator</uri>
</include>

<model name="coupler_model">
    <joint name="coupler" type="fixed">
        <!-- NG: Parent link must belong to coupler_model or be "world" -->
        <parent>vehicle::body</parent>
        <child>manipulator::base</child>
    </joint>
</model>
```
The following workaround on the final block is possible but not smart as an extra link and joint are required.
```xml
<model name="coupler_model">
    <!-- OK?: An extra link -->
    <link name="dummy">
        <inertial>
           <mass>0.1</mass>
           <inertia>
               <ixx>0.01</ixx>
               <iyy>0.01</iyy>
               <izz>0.01</izz>
           </inertia>
        </inertial>
    </link>
    <joint name="coupler1" type="fixed">
        <parent>dummy</parent>
        <child>vehicle::body</child>
    </joint>
    <!-- OK?: An extra joint -->
    <joint name="coupler2" type="fixed">
        <parent>dummy</parent>
        <child>manipulator::base</child>
    </joint>
</model>
```