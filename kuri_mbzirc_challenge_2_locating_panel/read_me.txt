The whole panel localization process can be divided into two parts:
---------------------------------------------------------------------------------------------------
1. Husky Navigation
consists of:
husky_wp node: Navigate the Husky through fixed waypoints until the goal point for the panel is set.
---------------------------------------------------------------------------------------------------
1. Search for panel 
consists of:
detection_velo node: Process the raw data from the Velodyne and publish the possible goal points at each time stamp.
detect_goal node:    Subscribe the possible goal points at each stamp and set the actual goal point to stop the fixed waypoint navigation executed by husky_wp node.
---------------------------------------------------------------------------------------------------
To recreate the demo in videos:


In a new terminal window, SSH to pr2-head:

Connect to the Velodyne
---------------------------------------------------------------------------------------------------
sudo ssh pr2-head@192.168.1.19
sudo ifconfig eth1 192.168.1.70
sudo route add 192.168.1.201 eth1
roslaunch velodyne_pointcloud VLP16_points.launch
---------------------------------------------------------------------------------------------------
In a new terminal window, SSH to pr2-head:
Start the "Search for panel" part
---------------------------------------------------------------------------------------------------
sudo ssh pr2-head@192.168.1.19
roslaunch detect_goal velodyne_part.launch
---------------------------------------------------------------------------------------------------
In a new terminal window, SSH to pr2-head:
start the other part of the "Search for panel" part
---------------------------------------------------------------------------------------------------
sudo ssh pr2-head@192.168.1.19
rosrun detect_goal detect_goal_point
---------------------------------------------------------------------------------------------------
In a new terminal window, SSH to the Husky onboard computer:
Start the "Husky Navigation" part
---------------------------------------------------------------------------------------------------
sudo ssh administrator@192.168.1.11
Two options:
1. Make Husky wait at the same place until the goal point is set
roslaunch husky_wp husky_nav_part_standby.launch
2. Let Husky navigate through fixed waypoints until the goal point is set
roslaunch husky_wp husky_nav_part_straight.launch

