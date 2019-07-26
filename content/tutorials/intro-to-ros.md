---
title: "Intro to ROS"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
difficulty: Beginner
duration: 60
summary: Learn basic ROS concepts.
weight: 2
---

## Introduction

### Goal
This tutorial will help you get familiar with ROS concepts in reference to the MuSHR software stack. Afterwards you should start to become comfortable with ROS [Publishers and Subscribers](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers).

### Prerequisites
In order to successfully complete this tutorial you will need: 

1. to have a working instalation of ROS. (get a link to relevant installation tools. quickstart?)
+  to have completed the [quickstart](/tutorials/quickstart) tutorial.
+  (*should*) have familiarity with `bash` and `python`.


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
{{< highlight xml "linenos=table" >}}
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
{{< / highlight >}}

The launch tags:
```xml
<launch> ... </launch>
```
are required preable to wrap any content in your launch files.

`<arg ...>` tags allow you to pass arguments in from the command line (or from other launch files). The `default` attribute specifies a default if no argument is passed in. To change an argument at runtime from the command line use the follwing syntax:
```bash
$ roslaunch <package> <launch file> plan_file:='/path/to/plan.txt'
```
It is good practice to use argument for state that can be set at runtime so users can choose values that make sense for their applications. It is also good practice to provide sensible defaults. If there is no sensible default, you should omit the default attribute. This will require the user to specify an argument at runtime.

In the third argument (line 4), the default uses a `roslaunch` command to locate a ROS package programatically. In order to keep your code portable, whenever you want to use files located in ROS packages, you can use the `$(find ...)` command to get the location of a ROS package. Then, no matter where the catkin workspace is located, `roslaunch` will populate the argument with the correct path.

The `<node ...>` tags denotes a single ROS node to be launched. ROS nodes are individual processes that run on a host. There are three attributes:

1. `pkg="mushr_ros_intro"`: The package to find the executable for the node.
+  `type="path_publisher.py"`: The entry file for the node. For python, you have to provide the name of the file in the `src` directory.
+  `name="path_publisher"`: The name of the node. This will be used for other nodes to reference your node. For now we will just use the same name as the executable, as this is a uniquely identifying name.

Line 7-9 define parameters for the node. Parameters are accessed programatically by the node (think `rospy.get_param(...)`). This is different than an argument, which is only used by `roslaunch` to pass values into the launch file. It is often convenient to use the same names for arguments and parameters, although this is entirely up to you.

## Putting it all together

With python files, we have to change the execution permissions so `roslaunch` can file the correct files to run.
```bash
$ chmod a+x src/path_publisher.py
```

"`chmod`" stands for change mode, "`a+x`" says, "let all users execute this program". **If you don't do this**, you will likely see the following message:

```txt
ERROR: cannot launch node of type [mushr_ros_intro/path_publisher.py]: can't locate node [path_publisher.py] in package [mushr_ros_intro]
```
So make sure to make your files executable!

Once this is done, all that's left to do is launch the file and the simulator. In one terminal run the simulator:
```bash
$ roslaunch mushr_sim teleop.launch
```

In another, start `rviz` (a ROS tool that allows you to visualize simulated environments):
```bash
$ rosrun rviz rviz -d $MUSHR/mush_utils/rviz/mushr_ros_intro.rviz
```
This will launch rviz with a configuration that has all the right topics visualized.

Now, finally, in another terminal, run the path publisher we created:
```bash
$ roslaunch mushr_ros_intro path_publisher.launch
```

This will start the path publisher immediately, so make sure you are watching the `rviz` screen.

## Wrap-up
This concluded the introductory tutorial. This tutorial was meant to get you hands on experience with both ROS and the MuSHR environment. This means many of the topics were glossed over in order to make the tutorial managable. If you are interested in diving deeper, have a look at the [ROS tutorials](), and then try the challenge problems below.


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
