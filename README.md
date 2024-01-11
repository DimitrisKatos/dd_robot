# Two wheel robot with Urdf and ROS 2

In this tutorial, you will embark on the journey of creating your inaugural ROS 2 package, delving into the realm of robotics by constructing your very first two-wheel robot. The exploration extends beyond mere assembly as you delve into the intricacies of building a comprehensive robotics system, utilizing URDF from the ground up. Additionally, you will gain proficiency in crafting launch files through Python scripting. Culminating in a visual and interactive experience, you'll leverage Rviz and the venerable Gazebo Classic to observe and engage with your robotic creation.

--- 

Before creating your first ROS 2 package you must install ROS2, install the colcon and create a ros workspace. You can follow the instraction in the following links:
- [Install ROS 2 humble.](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Install colcon to build packages.](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [Create a ROS 2 workspace.](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

In the following link, you can discover a presentation on the installation process of ROS 2 and the utilization of the colcon tool. The presentation also guides you through the establishment of a ROS 2 workspace. Furthermore you can cope with some ROS 2 feautures. 

- [Install ROS 2, Create ROS worksace.](https://docs.google.com/presentation/d/1xh91gPjNtocPdO5_trJKLcAcBAth0zTl/edit?usp=drive_web&ouid=106628092038381749227&rtpof=true) Greek Language.
- Install ROS2, Create ROS workspace. English Language. (comming soon)


## Create your first ROS 2 package.
We wiil create a package based on Python. The name of package is dd_robot:
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python dd_robot
```
For package organization we create the folders launch, urdf and wordls.
```
cd ~/ros2_ws/src/dd_robot
mkdir launch, urdf, worlds
```
For building the packages in ROS 2 you must informed the setup.py file. In  this file you should make the following changes.At first, add two python libraries. 
``` py
import os
from glob import glob 
``` 
Also you must add the following paths to the data_files Python list.
```py
(os.path.join('share',package_name,'launch'),
         glob(os.path.join('launch','*.launch.py'))),
(os.path.join('share',package_name,'urdf'),
         glob(os.path.join('urdf','*.urdf'))),
(os.path.join('share',package_name,'worlds'),
         glob(os.path.join('worlds','*.world'))),
```
The setup.py file of your package should look like this.

![Poll Mockup](./images/image1.png)

The following step is the building of the package.

```
cd ~/ros2_ws
colcon build packages-select dd_robot
```
## Create your first robot model 
In this chapter you will create your first robot model and you can check the behavior of the robot in Rviz. Also a laucnh file is introduced and by using them, the robot will be insert into Rviz.

Create a file named dd_robot.urdf in the folder urdf. 
```xml
<?xml version='1.0'?>
<robot name="dd_robot">
    <link name="base_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <box size="0.5 0.5 0.25"/>
            </geometry>
        </visual>
    </link>
</robot>
``` 
This code define a robot model name dd_robot. The robot has a link named base_link which is an box and its size is 0.5 width, 0.5 depth and 0.25 height.

create a python file with the name dd_robot_rviz.launch.py. and saved it to the launch folder.

`pip install math`

```py
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    dd_robot_path = get_package_share_path('dd_robot')
    default_model_path = dd_robot_path / 'urdf/ddrobot.urdf'
    

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    # Define all the Nodes which will start
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # You can use either joint state publisher or joint state publisher gui.
    # When you launch this file, give an extra argument.
    # gui = True for joint state publisher gui, False for joint state publisher. 
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
```
The next step is building the package, source the environment and launch the dd_robot_rviz.launch.py file.
```
cd ~/ros2_ws
colcon build --packages-select dd_robot
source install/setup.py
ros2 launch dd_robot dd_robot_rviz.launch.py
```
The launch file will begin the Rviz2 gui. To see your robot make the following step:

- Select Add in the Displays Panel. 
- From the new gui select RobotModel.
- From the Displays Panel, select topic and change to /robot_description
- Change the Fixed Frame to base_link


![Poll Mockup](./images/image2.png)

## Improve robot model 







---

## why graph
- help me
- okeii

---