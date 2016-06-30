#!/usr/bin/env python
import sys, rospy, tf, moveit_commander, random
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64

class UR5Wrapper:
  def __init__(self):
    self.manipulator = moveit_commander.MoveGroupCommander("ur5_arm")

  def setPose(self, x, y, z, phi, theta, psi):
    orient = \
      Quaternion(*tf.transformations.quaternion_from_euler(phi,theta,psi))
    pose = Pose(Point(x, y, z), orient)
    self.manipulator.set_pose_target(pose)
    self.manipulator.go(True)
    
if __name__ == '__main__':
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('ur5_cli',anonymous=True)
  argv = rospy.myargv(argv=sys.argv) # filter out any arguments used by ROS
  if len(argv) != 7:
    print "usage: ur5_cli.py XYZ phi theta psi"
    sys.exit(1)
  ur5w = UR5Wrapper()
  ur5w.setPose(*[float(num) for num in sys.argv[1:]])
  moveit_commander.roscpp_shutdown()
