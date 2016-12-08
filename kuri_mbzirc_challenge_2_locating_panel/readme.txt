1. The driver for Velodyne VLP16 needs to be built from source.
   Please follow the introduction:
   http://answers.ros.org/question/226594/how-do-i-build-ros-vlp16-velodyne-driver-for-indigo-using-catkin/
2. Connect Velodyne VLP16 to the ethernet port 
3. Open a new terminal 
   Enter:
   sudo ifconfig eth0 192.168.3.100
   sudo route add 192.168.1.201 eth0
4. Launch the driver 
   roslaunch velodyne_pointcloud VLP16_points.launch   
5. /velodyne_points topic should be published 
--------------------------------------------------------------------------------------------------------
Package.1:
velodyne_detect_panel 

Usage:
Detect panel candidate in the field of view 

Description:
The package subscribes to /velodyne_points. The pointcloud is then processed by clustering, vetical plane fitting, and couple point selections
The package publishes /panel_cloud. The pointcloud consists of points from panel candidate in the field of view.
The package publishes /center_cloud. The pointcloud consists of the center points of the panel candidates.

Run command:
rosrun velodyne_detect_panel detection
--------------------------------------------------------------------------------------------------------
Package.2:
velodyne_goal_point_selection

Usage:
Find the most stable panel candidate among the others.

Description:
The package subscribes to /center_cloud. The pointcould is first transformed from the local frame to the global frame. 
The poins from each frame is accumulated in a pointcloud in the global frame   /goal_point_candidates.
To find the most stable panel candidate, /goal_point_candidates is clustered.  (unfinished)
The pointcloud first has points over n=20 is considered as the "true" panel location.  (unfinished) 
The center of the pointcloud is sent to move_base as the goal point.  (unfinished)
--------------------------------------------------------------------------------------------------------
