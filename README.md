
# UR3_embedded_gripper_85_robotiq
This project has been developed with the goal of creating a system to control the cobot in simulation. Here the Cobot UR3 Series with the attached gripper , simulated in rviz and gazebo, can be controlled from Rviz or as well from the terminal by using scripts written in python.

![rviz_visualization](https://github.com/trabelsim/UR3_with_Robotiq_gripper_85/blob/master/gripper85.png)

## Requirements
ROS version : Melodic

O.S : Ubuntu 18.04

# Getting started

## Installation
### Set-up the workspace and install universal_robot package
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
In order to download the 3D model of the gripper follow the steps in the screenshot below (in the Download file of the link):
![gripper_download_85](https://user-images.githubusercontent.com/37307764/157120630-d0452c47-6cbc-4f64-b0f8-b2507060d11a.png)


<!-- In the download tab select Universal Robots, then Software, Gripper Software and in the end choose the Universal Robots URCAP (UCG-1.81 for Polyscope 3.10+ / 5.4+) and click on Download ZIP. -->
The downloaded file is .STEP format and contains the 3D model of the gripper which has to be converted before attaching to the robot.

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

#In my case the file is named "efektor_otwarty.stl"
#replace the effector_name with the name of the effector you converted already and copy it into ur_description/meshes/ur3/collision folder
cp efektor_otwarty.stl ~/catkin_ws/src/universal_robot/ur_description/meshes/ur3/collision

#the same has to be done for the visual state, but this time for the visual folder.
cp efektor_otwarty.stl ~/catkin_ws/src/universal_robot/ur_description/meshes/ur3/visual
```

### Create the gripper xacro file
Now that we added our 3D models to the workspace, we need to call them and attach to the appropriate place.
For this scope we need a xacro file which will manage the gripper separately. Then in the next steps we will combine the gripper xacro file to the robot xacro file.

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
  <xacro:macro name="measurement_tool" params="prefix connected_to">
    <!-- Create a fixed joint with a parameterized name. -->
    <joint name="${prefix}measurement_tool_joint" type="fixed">
      <!-- The parent link must be read from the robot model it is attached to. -->
      <parent link="${connected_to}"/>
      <child link="${prefix}measurement_tool"/>
      <!-- The effector has to be attached with the following parameteres,otherwise it won't be at the effector position. -->
      <origin rpy="1.5708 0 0" xyz="-0.061 0.076 -0.001"/>
    </joint>
    <link name="${prefix}effector_name_link">
      <visual>
        <geometry>
          <!-- The path to the visual meshes in the package. -->
          <mesh filename="package://ur_description/meshes/ur3/visual/efektor_otwarty.stl" scale = "0.0008 0.0008 0.0008" />
        </geometry>
      </visual>
      <collision>
        <geometry>
          <!-- The path to the collision meshes in the package. -->
          <mesh filename="package://ur_description/meshes/ur3/collision/efektor_otwarty.stl" scale = "0.0008 0.0008 0.0008" />
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

At this time we can move on and combine the gripper xacro file with the robot xacro file.

### Modify the ur3_robot.urdf.xacro file

Now we need to add the file we have just created to our main xacro file.
Basically the ur3_robot.xacro file gets the information from various .xacro and then load them in order to run them together.

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
Now we need to generate our urdf (it means that the structure will be compiled) , so we will need to call the following command : 
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

