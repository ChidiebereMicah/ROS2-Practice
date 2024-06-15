# ROS2-Practice

The codes in this repo are ROS2 python nodes and should be executed as such.

To run them as ROS2 nodes, they have to be in a ROS2 package which should be in a ROS2 workspace.
The nodes should be located in this directory: \\home\User\ros2_ws\src\<your_package_name>\<your_package_name>

These dependencies are to be added to the package.xml file in the src folder in the workspace 
  '''<depend>rclpy</depend>
     <depend>geometry_msgs</depend>
     <depend>turtlesim</depend>
     <depend>math</depend>'''

The location of the package.xml file is \\home\User\ros2_ws\src\<your_package_name>\package.xml

Remember to add the node to the setup.py file in \\home\User\ros2_ws\src\<your_package_name>\setup.py
Nodes are added here:
...
...
'console scripts': ["move_to_place = <your_package_name>.move_to_place:main",
                       "turtle_love2 = <your_package_name>.turtle_love2:main"],
...
...
