ROS Navigation Stack
====================

A 2D navigation stack that takes in information from odometry, sensor
streams, and a goal pose and outputs safe velocity commands that are sent
to a mobile base.

 * AMD64 Debian Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__navigation__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__navigation__ubuntu_bionic_amd64__binary/)

Related stacks:

 * http://github.com/ros-planning/navigation_msgs (new in Jade+)
 * http://github.com/ros-planning/navigation_tutorials
 * http://github.com/ros-planning/navigation_experimental

For discussion, please check out the
https://groups.google.com/group/ros-sig-navigation mailing list.

=======================================================================================
23 June 2021 by hkm

To overay your custom navigation stack on the standard ros pack installed in your PC

    1. make sure to remove out  "source catkin_ws/devel/setup.bash" in your bashrc
    2. then, open a prompt, go to catkin_ws followed by "source /opt/ros/<version>/setup.bash"
    3. do catkin_make to recompile your navigation stack
    4. finally, run "source catkin_ws/devel/setup.bash"
    5. try roscd to your custom overayed nav stack. 



