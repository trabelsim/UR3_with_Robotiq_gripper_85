
# UR3_embedded_gripper_85_robotiq
This project has been developed with the goal of creating a system to control the cobot in simulation. Here the Cobot UR3 Series with the attached gripper , simulated in rviz and gazebo, can be controlled from Rviz or as well from the terminal by using scripts written in python.

## Requirements
ROS version : Melodic

O.S : Ubuntu 18.04

# Getting started

## Installation
### Set-up the workspace and universal_robot
* [universal_robot](https://github.com/ros-industrial/universal_robot) - ROS-Industrial

```
sudo apt-get install ros-$ROS_DISTRO-universal-robot
```

replace ```$ROS_DISTRO``` with ```melodic```
```
mkdir -p ~/catkin_ws/src

# again here - replace $ROS_DISTRO with the ROS version used
git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

cd $HOME/catkin_ws

#checking dependencies
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

#build the workspace
catkin_make (or catkin build)

#source the workspace so you can start using it
source $HOME/catkin_ws/devel/setup.bash
```
At this moment we have set-up our UR environment, which can be already used but without a gripper.

### Download the gripper

[2F-85 Gripper](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button#support-documents-no-auto-scroll)

The downloaded file has the format of .STEP.

**In order for the xacro to be processed the format has to be CONVERTED IN .STL OR .DAE**

At this time you can use free online converters, freecad or autocad in order to complete this task.

### Add the gripper model to the workspace
When adding a model into ROS, it has to be splitted in two different sections :

* visual

* collision

For more information about differences [here](https://answers.ros.org/question/304171/what-is-self-collide-and-what-is-difference-between-visual-and-collision/)

So we will need to add the model to both of them (visual and collision).
```
#move to the path with the .stl/.dae file 
cd /path/where/the/file/has_been_saved_and_converted.stl

#replace the effector_name with the name of the effector you converted already
cp effector_name.stl ~/catkin_ws/src/universal_robot/ur_description/meshes/ur3/collision

#the same has to be done for the visual state
cp effector_name.stl ~/catkin_ws/src/universal_robot/ur_description/meshes/ur3/visual
```
    
### Create the gripper xacro file
Let's first move to the folder where the urdf files are stored.

```
cd ~/catkin_ws/src/universal_robot/ur_description/urdf
gedit gripper.xacro
```
Here we need to create a file xacro which will be generate our model by the input parameters.
Paste the following code inside : 
```
<?xml version="1.0" encoding="utf-8"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- Here we define the 2 parameters of the macro -->
  <xacro:macro name="effector_name_X" params="prefix connected_to">
    <!-- Create a fixed joint with a parameterized name. -->
    <joint name="${prefix}effector_name_joint" type="fixed">
      <!-- The parent link must be read from the robot model it is attached to. -->
      <parent link="${connected_to}"/>
      <child link="${prefix}effector_name"/>
      <!-- The tool is directly attached to the flange. -->
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </joint>
    <link name="${prefix}effector_name_link">
      <visual>
        <geometry>
          <!-- The path to the visual meshes in the package. -->
          <mesh filename="package://ur_description/meshes/ur3/visual/effector_file.stl" scale="0.001 0.001 0.001 />
        </geometry>
      </visual>
      <collision>
        <geometry>
          <!-- The path to the collision meshes in the package. -->
          <mesh filename="package://ur_description/meshes/ur3/collision/effector_file.stl" scale="0.001 0.001 0.001 />
        </geometry>
      </collision>
    </link>

    <!-- TCP frame -->
    <joint name="${prefix}tcp_joint" type="fixed">
      <origin xyz="0 0 0.116" rpy="0 0 0"/>
      <parent link="${prefix}effector_name"/>
      <child link="${prefix}tcp"/>
    </joint>
    <link name="${prefix}tcp"/>

  </xacro:macro>
</robot>
```
Save the file and close it.

### Modify the ur3_robot.urdf.xacro file

Now we need to add the file we have just created to our main xacro file.
Basically the ur3_robot.xacro file gets the information from various .xacro and then load them in order to run together.

```
gedit ur3_robot.urdf.xacro
```

You should see something similar to this : 
```
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro"
       name="ur3" >

  <xacro:arg name="transmission_hw_interface" default="hardware_interface/PositionJointInterface"/>

  <!-- common stuff -->
  <xacro:include filename="$(find ur_description)/urdf/common.gazebo.xacro" />

  <!-- ur3 -->
  <xacro:include filename="$(find ur_description)/urdf/ur3.urdf.xacro" />

  <!-- arm -->
  <xacro:ur3_robot prefix="" joint_limited="false"
    transmission_hw_interface="$(arg transmission_hw_interface)"
  />

  <link name="world" />

  <joint name="world_joint" type="fixed">
    <parent link="world" />
    <child link = "base_link" />
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
  </joint>

</robot>
```
After
```
<!-- ur3 -->
  <xacro:include filename="$(find ur_description)/urdf/ur3.urdf.xacro" />
```
add the following text : 
```
  <!-- end-effector -->
  <xacro:include filename="gripper.xacro" />
  <xacro:effector_name prefix="" connected_to="tool0"/>
```

And save it.
Now we need to generate our urdf, so we will need to call the following command : 
```
rosrun xacro xacro -o gripper.urdf gripper.xacro
```
In the end we need to catkin_make and source the path of our catkin_ws package
```
cd ~/catkin_ws
catkin_make   # or catkin build
source devel/setup.bash
```

The only thing that remain to be done is Launch it!

For this step I developed a little script that you can find [here](https://github.com/trabelsim/UR3-App-Script) , it allows to run in a row all the applications that you need to perform tasks : Gazebo , MoveIt, RviZ.


### Author 
Maroine Trabelsi , Wrocław University of Science and Technology
