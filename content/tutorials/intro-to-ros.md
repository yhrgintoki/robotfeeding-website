---
title: "Intro to ROS"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
difficulty: Beginner
duration: 120
summary: Learn basic ROS concepts.
weight: 2
---

## Introduction

### Goal
This tutorial will help you get familiar with ROS concepts in reference to the MuSHR software stack. Afterwards you should start to become comfortable with ROS [Publishers and Subscribers](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers).

### Prerequisites
In order to successfully complete this tutorial you will need to have completed the [quickstart](/tutorials/quickstart) tutorial.
1. Must have a working instalation of ROS. (get a link to relevant installation tools. quickstart?)
+  Must have installed the Simulator
+  Must have completed the quickstart in sim.
+  Should have familiarity with `bash` and `python`.


### Notes
This tutorial assumes your catkin workspaces is located at `~/catkin_ws`. If you followed the [quickstart](/tutorials/quickstart) tutorial, this will be the case. If your workspace is in a different directory, adjust the provided command accordingly.

## Creating a package

First start by creating a catkin package for our code:

```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg mushr_ros_intro std_msgs rospy ackermann_msgs
```

Our package depends on `std_msgs`, `rospy`, and `ackermann_msgs`. All new ROS packages should depend on `std_msgs` and `rospy` (if you plan to use C++, then your package should also depend on `roscpp`. We include `ackermann_msgs` because it is the way we send velocities and steering angles to be applied on the car (more on this later in the tutorial).

Now we will build our empty package:

```bash
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roscd mushr_ros_intro
```

Sourcing `devel/setup.bash` sets up the ROS environment as well as sets up some useful auto-complete rules, easing the traversal of multiple ROS packages. We can see this power with the `roscd` command. Once you use the `roscd` with `mushr_ros_intro` once, you should be able to tab complete the name. *Sometimes, when you encounter packages not being found, all you need it to rerun the source command.*

## Writing the node

Now, that we have a package, we want create a source file to run our node. We will be creating a simply ROS node to read commands (velocity and steering angle) from a file line by line, and sending them to the simulator to be applied to the simulated car. Each line will denote a command to be applied for a second. The first line is a message to send as the "starting pose" of the car. The input files will be of the form:

```
0,0,0.0
2.0,0.09
3.0,-0.15
```

The first line is the initial position, of the form `x, y, theta`, where `x` and `y` are the starting coordinates in the map, and `theta` is the initial angle of the car. The following two commands tell the car how fast to go, and at what steering angle. The first to run at `2.0 meters per second`, with a steering angle of `0.09 radians`. The second, to run at `3.0 meters per second`, with a steering angle of `-0.15 radians`. We will be applying each command for 1 second. A positive steering angle corresponds to a left turn and a negative steering angle corresponds to a right turn.

Below the entire code is listed. Each section will be explained in greater detail below the listing. Save this file in `src/path_publisher.py`


{{< highlight python "linenos=table" >}}
#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
)
from tf.transformations import quaternion_from_euler


def run_plan(pub_init_pose, pub_controls, plan):
    init = plan.pop(0)
    send_init_pose(pub_init_pose, init)

    for c in plan:
        send_command(pub_controls, c)


def send_init_pose(pub_init_pose, init_pose):
    pose_data = init_pose.split(",")
    assert len(pose_data) == 3

    x, y, theta = float(pose_data[0]), float(pose_data[1]), float(pose_data[2])
    q = Quaternion(*quaternion_from_euler(0, 0, theta))
    point = Point(x=x, y=y)
    pose = PoseWithCovariance(pose=Pose(position=point, orientation=q))
    pub_init_pose.publish(PoseWithCovarianceStamped(pose=pose))


def send_command(pub_controls, c):
    cmd = c.split(",")
    assert len(cmd) == 2
    v, delta = float(cmd[0]), float(cmd[1])

    dur = rospy.Duration(1.0)
    rate = rospy.Rate(10)
    start = rospy.Time.now()

    drive = AckermannDrive(steering_angle=delta, speed=v)

    while rospy.Time.now() - start < dur:
        pub_controls.publish(AckermannDriveStamped(drive=drive))
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("path_publisher", anonymous=True, disable_signals=True)

    control_topic = rospy.get_param("~control_topic", "/mux/ackermann_cmd_mux/input/navigation")
    pub_controls = rospy.Publisher(control_topic, AckermannDriveStamped, queue_size=1)

    init_pose_topic = rospy.get_param("~init_pose_topic", "/initialpose")
    pub_init_pose = rospy.Publisher(init_pose_topic, PoseWithCovarianceStamped, queue_size=1)

    plan_file = rospy.get_param("~plan_file")

    with open(plan_file) as f:
        plan = f.readlines()

    # Publishers sometimes need a warm-up time, you can also wait until there
    # are subscribers to start publishing see publisher documentation.
    rospy.sleep(1.0)
    run_plan(pub_init_pose, pub_controls, plan)

{{< / highlight >}}

## Writing the launch file

In order to run out code in a convenient and extendable way, ROS has the notion of launch files. These are XML files that describe how different software components should be started. These file allow us to start a large number of ROS nodes with few commands. We will write a launch file for our node to conform to this standard. First we make a launch directory:
```bash
$ roscd mushr_ros_intro # if you aren't in the directory
$ mkdir launch
```

Create the file `launch/path_publisher.launch`, containing:
```xml
<launch>
    <arg name="control_topic" default="/mux/ackermann_cmd_mux/input/navigation" />
    <arg name="init_pose_topic" default="/initialpose" />
    <arg name="plan_file" default="$(find mushr_ros_intro)/plans/straight_line.txt" />

    <node pkg="mushr_ros_intro" type="path_publisher.py" name="path_publisher" output="screen">
        <param name="control_topic" value="$(arg control_topic)" />
        <param name="init_pose_topic" value="$(arg init_pose_topic)" />
        <param name="plan_file" value="$(arg plan_file)" />
    </node>
</launch>

```

## Putting it all together

Need to chmod!!!
Launch the sim, launch rviz, launch this bad boy.

## Challenges
1. create new paths!
+  use a different map!
+  instead of requiring the enire file path to our plan, take then plan name as an argument, and look in the `paths` directory.
+  adapt `path_publisher` to take a "duration" parameter as the third comma separated value.
+  when the plan is done being executed, restart from the the begining until the node is quit


<!--
Create a new package `mushr_ros_intro`, depending on `roscpp, rospy, std_msgs, ackermann_msgs` (if we don't have `ackeramnn_msgs`, have to get it via `apt install ros-melodic-ackermann-msgs`)
+   Run the commands
```
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roscd mushr_ros_intro
```
The first three build all the packages and setup the environment. The final command changes the current working directory to the package we just created.
+  Run commands
```
$ mkdir src
$ mkdir launch
```
Explain what these two directories will contain. src source files, launch launch files to run our scripts in the ROS environment.
+  Create a file `src/path_pub.py`. This script will read a list of commands from a file 
-->
