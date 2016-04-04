# MBZIRC Challenge 2

This package is developed based on the jsk simulation environemnt http://github.com/start-jsk/jsk_mbzirc

## Installing

Follow the steps below to install the simulation environment with all it's dependencies.

```
cd <catkin_ws>
wstool init src
wstool set -t src kuri_mbzirc_challenge_2 https://github.com/kuri-kustar/kuri_mbzirc_challenge_2.git --git
wstool merge -t src https://raw.githubusercontent.com/kuri-kustar/kuri_mbzirc_challenge_2/master/mbzirc.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

