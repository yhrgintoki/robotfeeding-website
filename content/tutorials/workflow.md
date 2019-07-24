

# MuSHR Workflow Tutorial
**Description:** You will learn the basic components of the car and their associated software counterparts, how to work with the car, and other tips and tricks that will make your life much easier. We will also cover how to troubleshoot issues. This tutorial is structured to be largely informative, we recommend you follow along with the workflows and troubleshooting to see the steps in person.

### Contents
1. [Car Overview](#car-overview)
2. [Common Workflows](#common-workflows)
3. [Troubleshooting](#troubleshooting)

## Car Overview
This section will cover an overview of the physical racecar and the associated software components. Below is a diagram of the physical racecar components.

<img src="https://gitlab.cs.washington.edu/cse490r/19sp/mushr_workflow_tutorial/raw/master/hardware.jpg" width=800px>

### Hardware Components
Each version of the racecar will have a slightly different setup, but in essence they all follow this diagram. Here is a brief description of each of these components and their function:

* __Computer (Jetson TX2):__ This the computer that runs the software on the car. You can connect to the computer in 2 ways primarily. If it does not have an internet connection with a static IP then we can use the provided HDMI/USB ports to plugin a keyboard/mouse/monitor. We set the cars up to have a static IP (associated with the car #) so we can use `ssh` easily and leverage our base computer's (laptop/desktop) monitor/keyboard to work with the computer. see [Troubleshooting](#troubleshooting) for more info on `ssh`. 
* __Motor:__ While the motors may vary, all the cars have a single DC motor that powers all four wheels. The motor makes the car move and is controlled by the VESC.
* __Servo:__ A servo is another type of motor but is better at going to specific angles along its rotation than rotating continuously. The servo's job is the steer the front wheels by taking in steering angle commands. The servo is also controlled by the VESC.
* __VESC:__ The VESC is responsible for taking high level control commands like steering angle and velocity and converting that to power/angle commands for the motor/servo. The VESC has an associated ROS node. This node takes your ROS `ackermann_msgs/AckermannDriveStamped` message and converts that to `VescStateStamped` (power), and `Float64` (steering angle) messages. These messages are something the physical VESC can use to control the motor and servo.
* __NiMH Battery:__ On most versions, there are 2 batteries. This specific one is meant to power the motor and in turn the VESC (because the motor power flows through the VESC). A lot of issues (particularly with the VESC) can stem from one of these motors having a low battery, so it is good to check it regularly.
* __Battery Bank:__ This battery powers everything else: computers, sensors, and the usb hub. There are 3 different versions of these. The old ones which are relatively plug and play. The ones with the power button (magic button) on the side. And the ones with the power on the front. For troubleshooting battery issues see  [Troubleshooting](#troubleshooting).
* __RGBD Camera:__ This Intel realsense camera can publish both rgb and depth. It has a associated node so you can just subscribe to the topic to use the images! OpenCV even has a function for converting ROS `Image` messages to something OpenCV can use. Note depth cameras are different from stereo cameras (how humans do it). They both provide the same output, but depth cameras project a IR pattern and use a IR camera to see the deformation of that pattern to compute depth. Depth cameras will also sometimes use stereo in addition to make their measurements more accurate. It publishes to the `/camera` topics.
* __Laser Scanner:__ This sensor works by having a 1D laser range finder spin around. With each distance reading their is a associated angle the range finder was at. It publishes an array of distance and angle measurements to the `/scan` topic.
* __IMU:__ Inertial measurement unit. This is used to provide odometry measurements, but in reality it is not being used by the current system. It publishes on the `/imu` topic.
* __Wireless Controller:__ This provides controls to the cars! It doesn't just have to be use for teleop it can be used by your programs for most anything. It publishes on the `/joy` topic.
* __Bumber Switch:__ This is a button on the version 2 cars that you can use to indicate a collision. It publishes a binary signal **button topic needed**.
* __SSD Card:__ Large storage for bagfiles and other data collection. You can find the directory for it under `/media/JetsonSSD`.
* __USB Hub:__ The Jetson doesn't have enough USB ports so we have a powered USB hub to connect the lidar, camera, and IMU. Since it is powered (meaning it has it takes in additional power besides the power provided over USB) it needs to be plugged into the Battery Bank for messages to pass through it, a good thing to know if you get connection issues.

 ### Software Components
 Now that we have a better understanding of the hardware components let's checkout the software components and see how it all ties together. Let's start by connecting to the car:  
 `ssh nvidia@172.16.77.##`
 Then go to your catkin workspace  
 `cd ~/catkin_ws/`
 and list the directories
 ```
 nvidia@louiseii:~/catkin_ws$ ls
build  devel  src
 ```
 So what we have is 3 directories. `build` is where code is compiled to. `devel` has setup files for your environment. The most important thing about `devel` is that it contains `setup.**`. 
 ```
 nvidia@louiseii:~/catkin_ws/devel$ ls
bin  env.sh  include  lib  setup.bash  setup.sh  _setup_util.py  setup.zsh  share
 ```
 So `setup.**` (** because you can run .bashr, .sh, or .zsh and get the same effect) sets up ROS environment variables and paths. So if you ever see a error that says that it can't find a package, or that roscore isn't a command, it is because you have not sourced your `setup.**` like so
 `source ~/catkin_ws/devel/setup.bash`
We usually prevent this by putting the above in your `~/.bashrc` a file that is run everytime you log in. 
So now if you changed directories to `~/catkin_ws/src/` you will see all the ros packages. When you want to make new ROS software you make a package and you put it here. Why? Because when you run `catkin_make` it looks in this directory for the packages it needs to build. `catkin_make` needs to be run in the `~/catkin_ws/` directory and should be run after every update in code. Although, since python is an interpreted language, we do not have to run `catkin_make` everytime we edit code, but in C++ we would.

Alright, now that we have a high level view of our workspace let's checkout the system components. Each package (located in `src`) creates ROS nodes. This tutorial will not dive into the details of how ROS works but this [post](https://robohub.org/ros-101-intro-to-the-robot-operating-system/) gives a good overview with diagrams. Below is a simplified diagram of the system at a high level:

<img src="https://gitlab.cs.washington.edu/cse490r/19sp/mushr_workflow_tutorial/raw/master/racecar_overview.png" width=800px>

Let's unpack this, starting from the top. The sensor nodes take the raw input, do some processing, and  convert it to ros friendly topics. The map server converts a `.yaml` map file to a ros topic. Your controller (orange) takes in sensor topics coming in from the each sensors' ros node and the map. It then outputs some command to drive the car. That command is put into a high level mux which listens on multiple `.../nav_*` channels and selects the highest priority (nav_0). `.../default` is a zero throttle/steering command that is passed whenever your controller is not publishing. The high level mux is sent via the `.../output` topic.

The output of the high level mux gets sent to the low level mux which incorporates teleop and safety control inputs. These controls have a higher priority than your controller's nav messages. Currently, MuSHR does not have a explicit safety controller.

Once the highest priority command is output it goes to the vesc. The vesc smooths the command by clipping the min/max of the steering/throttle so we don't try to turn the wheels 180 degrees for example. It then provides that to the vesc driver which directly controls the motors. Something that can be confusing about this architecture is the /vesc namespace is being used for the mux (see the nodes/topic names) and some vesc code can be found in the mux. Cleanly separating the mux from the vesc is something we have fixed in new iterations of the codebase.

These components are in the following locations if you want to check them out for more details:

 - MUX:  `~/catkin_ws/src/racecar_base/ackermann_cmd_mux`
 - VESC: `~/catkin_ws/src/vesc`
		 - This is a meta-package with multiple sub directories
		 - `.../vesc_ackermann/` contains vesc odom info (not depicted in diagram)
		 - `.../vesc_driver/` is the last piece of the diagram connecting the physical vesc with the computer
		 - `.../vesc_msgs/` describes the VescState message
		 - `.../ackermann_cmd_mux/src/throttle_interpolator.py` is the throttle interpolator
- Teleop: `~/catkin_ws/src/racecar_base/racecar/scripts/joy_teleop.py`
- Lidar: `~/catkin_ws/src/racecar_base/ydlidar`
- Camera: `~/catkin_ws/src/realsense`
- Map: `~/catkin_ws/src/racecar_base/racecar/includes/common/map_server.launch`
## Common Workflows
There are two common workflows, in sim and in the real world. Sim is easier because you do not need to connect and communicate over the WIFI. We recommend getting comfortable working in sim before trying to test on the car.

### Simulation Workflow
To work in sim you need to have the mushr_sim and rviz installed along with racecar_base and its dependencies. The basic workflow is the following:
1. `roscore` - this starts the rosmaster that the other programs will latch to. If you don't run this then rviz can act up. Why? Because rviz needs a rosmaster to talk to and if the rosmaster is teleop, or some other node then when you restart, rviz will be lost.
2. `rosrun rviz rviz` - start rviz. Ideally you want to open your previously saved .rviz file (file -> save as) that has your most commonly used topics all setup. But if not then wait till everything is running, subscribe, then save a file. Also, since you are working with a 2D map, make sure the camera is set to TopDownOrtho.
3. `roslaunch mushr_sim teleop.launch` - launch the sim
4. `roslaunch mushr_sim map_server.launch` - launch map server. If you don't want to launch this everytime, then I would recommend adding the following line at the end of teleop.launch in mushr_sim: `<include file="$(find mushr_sim)/launch/map_server.launch" />` Then everytime you launch the sim the map server will also be launched!
5. `roslaunch your_package your_launchfile.launch` - Your code
6. Subscribe to all the necessary topics (and save a .rviz file!)
7. Use the gray box and the W, A, S, D keys to drive

If you need to restart/edit your code then make sure to reset the rviz topics so you get the most up to date data. When done, press `q` in the gray box. 

### Real World Workflow
Real world has a similar process to simulation, but with added hardware setup and connecting remotely. The key to a solid workflow, is to make sure to separate hardware and software failures clearly which is something that we will discuss more in the next section.

The first step is to make sure both batteries have sufficient charge (not dead at least). The computer batteries for all cars have LEDs that indicate how much charge they have. The Vesc batteries require that you plug them into a charger to check if they are charged. The below images show both old (left) and new (right) batteries in a charged state. On the old, the red solid dot indicates how charged it is (almost full). When charging the green light should blink (hold start/stop to toggle). Right the solid green status light indicates full, red would indicate charging (press start/stop to toggle)
<div><img  src="https://gitlab.cs.washington.edu/cse490r/19sp/mushr_workflow_tutorial/raw/master/20190508_102823.jpg" width=300px>
<img src="https://gitlab.cs.washington.edu/cse490r/19sp/mushr_workflow_tutorial/raw/master/20190508_102753.jpg" width=300px></div>

Once you have comfirmed your batteries are charged, you have eliminated many of the most common issues. The next thing to do is to plug in the batteries starting with the vesc. Once you have plugged both batteries in, you should see a red light (shown below) of the TX2 powered.

<img src="https://gitlab.cs.washington.edu/cse490r/19sp/mushr_workflow_tutorial/raw/master/20190508_102503.jpg" width=300px>

You should also see a blue Vesc light shown below.

<img src="https://gitlab.cs.washington.edu/cse490r/19sp/mushr_workflow_tutorial/raw/master/20190508_102427.jpg" width=500px>

Now it is time to turn on the computer and begin the software portion. Start the computer by pressing the power button on the TX2, you should see a green light. Following that, go to your laptop/desktop with ROS installed and ssh into your car:
```
ssh nvidia@172.16.77.##
``` 
where ## is the number on your car. If you are having trouble connecting see [Troubleshooting](#troubleshooting). We need to setup the ROS_IP and the ROS_MASTER_URI environment variables on both devices. ROS_IP tells your ROS node what IP to communicate under. localhost will not work because it will prevent remote components from communicating with it. So use `ifconfig` in the terminal to find your IP (car and desktop) and set that number to your ROS_IP. 
```
export ROS_IP=172.16.77.##
```
If your IP is static (the car should be) then you can put this command at the bottom of your `~/.bashrc` and it will run everytime you login. You can check if an environment variable is set using `echo`
```
echo $ROS_IP
```
Now that you have set your ROS_IP for both the car and computer, we need to set the ROS_MASTER_URI. The ROS_MASTER_URI tells your ROS nodes where to look for rosmaster. The default is localhost:11311. Since your program is running on the car we will set the ROS_MASTER_URI to the car. Luckly since the car is already set to itself, we only have to set the desktop/laptop. On the base computer run:
```
export ROS_MASTER_URI=http://172.16.77.##:11311
```
11311 is the port. If this isn't set your base computer will start a separate rosmaster and if it is set incorrectly it will throw a error that it cannot find rosmaster. Now we are ready to run stuff!
1. On the car: `roscore`
2. On the base: `rosrun rviz rviz` - setup should be the same as sim
3. On the car: `roslaunch racecar teleop.launch`
4. Make sure you can drive the car and steer
5. On the base: Visualize topics in rviz
6. On the car: `roslaunch your_package your_launch.launch` - It is good to keep your package separate from teleop so that you can restart your program without killing teleop.

## Troubleshooting
Troubleshooting is what 80% of a roboticist time is spent on. If we know it is inevitable, we need to design systems and use tools to narrow down a diagnosis for the problem as fast as possible. Diagnosis can usually be the hardest part because it could be hardware or software or both. In addition, a robotic system is highly interconnected so a weird behavior in one component may only manifest itself in another component down the road. This section will cover the main debugging tools you should use on the car and some common problems and fixes.

### Debugging Questions
The following questions should be answered in order to work towards a diagnosis.

#### Is the issue hardware or software?
Very important question, as your fix will change drastically. There are usually clues into this problem. Let's look at an example:
```
[FATAL] [1455208235.408745600]: Failed to connect to the VESC, SerialException Failed to open the serial port to the VESC. IO Exception (2): No such file or directory, file /tmp/binarydeb/ros-kinetic-serial-1.2.1/src/impl/unix.cc, line 151. failed..
```
Now this issue is hardware. There is a key clue here, the word "IO Exception." We know the Vesc is not connected because we can't start a serial connection with the Vesc. Here is another example:

```
ERROR: unable to contact ROS master at [http://172.16.77.06:11311]
The traceback for the exception was written to the log file
```
Now this is also a connection issue, but this time it is software. We know because it is trying to connect to rosmaster at the set IP. So in software we are setting this IP from the environment variable ROS_MASTER_URI, and the computer is trying to connect to this IP over wifi. Either we set the IP wrong, or the wifi isn't working. But we know the wifi is functional because we can ssh into the car, so it must be a incorrect IP!

#### What component is causing the issue?
So now that you have determined that it is hardware vs. software, we need to narrow down the problem. Sometimes, in the above examples, it explicitly tells you, but we aren't always that lucky. Take the following example:
```
[teleop.launch] is neither a launch file in package [labx] nor is [labx] a launch file name
The traceback for the exception was written to the log file
```
So this points to labx being the component (package in this case) that is not working. Turns out that is slightly a red herring. So this error means ROS cannot find the launchfile or the package. So it could be one of two things or both. `teleop.launch` is not in labx in which case labx was the culprit. And/or you haven't sourced your workspace since making this package so ROS does not have it in its package list. We can narrow this down by doing the following:
```
rospack find labx
/home/nvidia/catkin_ws/src/labx
```
It found the package! Which means you don't have a `teleop.launch` file in your labx package

#### Is the error ROS related or pure code related?
A helpful thing to determine is if the problem has anything to do with ROS. If the error looks like a standard python/C++ error then great, you can rule out all ROS stuff. If not, then it could be either your ROS interface (publishers/subscribers) in your code, or your launchfiles, or your ROS environment. This question is relatively easy to answer if you find the core issue, the one difficulty is that ROS will add a bunch of node failure gibberish even if your code logic is the problem. So just make sure to scroll the error to the very beginning to find the core issue.

### Debugging Tools
ROS provides a suite of tools to help debug issues. I'll cover each a bit and when to use.

- **rostopic**
This tool is really useful to checking if topics are publishing, get a sense of latency, see what is being published, and more info about specific topics.
	- `rostopic list` allows you to see all the topics
	- `rostopic echo topic_name` allows you to see what is actually being published
	- `rostopic info topic_name` lets you see the message type and other important info about a topic
	- ` rostopic hz topic_name` lets you see the publish rate of the topic. A quick way to detect a bottleneck.
-  **rosnode**
This tool works very similar to rostopic except on a node level. It is useful to see what nodes are publishing/subscribing to.
	- `rosnode list` list all the nodes
	- `rosnode info node_name` see what the node is publishing/subscribing to and other important info
- **rosparam**
If you have params that are set dynamically (erpm gain) then this a good tool to make sure a param is what you expect it to be and if not change it.
	-`rosparam list` list all params
	- `rosparam get param_name` get param value
	- `rosparam set param_name` set param value
- **rospack**
This tool is useful if you need to find a package.
	- `rospack list` list all packages
	- `rospack find package_name` gives you the location of the specified package if it finds it
- **rosrun tf**
If you are having transform issues, this tool is a good way to debug. 
	- `rosrun tf tf_echo frame1 frame2`  will output the transform from frame1 to frame2
	- `rosrun tf view_frame` will create a pdf diagram of the transforms present
	- `rosrun tf tf_monitor` show all frames and publish rates
- **rqt_graph**
This will give you a sense of the overall system of nodes and topics connecting them.
	- `rqt_graph` creates pdf of nodes and topics running

- **ssh**
While this is not really a debugging tool, it is a tool commonly used and should be touched on. ssh stands for Secure Shell. It is an encrypted network protocol that amongst other things gives you a shell session on a remote machine. It is super useful for remote work and connecting to robots because then you do not need an additional monitor and keyboard. So when you connect, you supply the username (nvidia) and the IP of the car and it will connect you to that user specifically. There are some other potentially useful things you can do with ssh in addition to a standard shell.  
	- `ssh nvidia@172.16.77.## -X` this connects a X session so you can run graphical applications remotely, but it will be very slow because everything needs to be encrypted
	- `sftp nvidia@172.16.77.##` secure file transfer protocol is the best way to get files from another machine. It allows you to `cd` and `ls` like a shell but also `get` files and `put` files from your local machine on the remote machine.

- **tmux**
The terminal multiplexer is a famous and commonly used tool for working with multiple shell sessions from one window. For us, this is particularly useful when you ssh into the car. You don't have to keep sshing in for each new window, but instead ssh in once then use tmux to have multiple sessions. This [tutorial](https://hackernoon.com/a-gentle-introduction-to-tmux-8d784c404340) goes into more detail.

### Common Issues & How to Fix
Alright, now that we know how to narrow down issues, let's look at the most common issues on the cars and how to fix them.

- **Vesc Failure**  
```
[FATAL] [1455208235.408745600]: Failed to connect to the VESC, SerialException Failed to open the serial port to the VESC. IO Exception (2): No such file or directory, file /tmp/binarydeb/ros-kinetic-serial-1.2.1/src/impl/unix.cc, line 151. failed..
```
**Hardware/Software:** Hardware  
**Component:** Vesc  
**ROS Related:** No, but it manifests itself through ROS  
**Fix:** Make sure the Vesc battery is charged and plugged in. The Vesc should have a blue light. If continues, check connections to the computer.  

- **ROS Workspace Not Setup (bash not recognizing ROS commands):**  
```
-bash: roscore: command not found
```
**Hardware/Software:** Software  
**Component:** ROS  
**ROS Related:** Yes  
**Fix:** `source ~/catkin_ws/devel/setup.bash`. We recommend putting this at the end of your `~/.bashrc` so you never experience this issue.  

- **Car Only Has Steering**  
**Hardware/Software:** Hardware  
**Component:** Vesc Battery  
**ROS Related:** No  
**Fix:** Charge the Vesc battery, it doesn't have enough juice to power the motor.  

- **Car Drifts When Driving Straight**  
This can also manifest when using a particle filter that works in sim but not on the car because the expected control is to go straight when commanded straight. It could also manifest in a controller, when the car cannot seem to follow a trajectory because what it is commanding to the car is not what the car is doing.  
**Hardware/Software:** Software  
**Component:** Vesc config  
**ROS Related:** No  
**Fix:** edit the `steering_angle_to_servo_offset` in  `~/catkin_ws/src/racecar_base/racecar/config/racecar-uw/vesc.yaml` to a value that when the car is commanded straight it goes straight. If using the old cars, adjust `.../config/racecar-mit/vesc.yaml`  

- **ROS Topics Not Appearing in rviz**  
**Hardware/Software:** Software  
**Component:** Your component/rviz  
**ROS Related:** Yes  
**Fix:** Either your component is not publishing the topic (use rostopic to double check!) or rviz needs to be refreshed or the transform is not publishing (rostopic to double check!). Click "Reset" in rviz and try restarting your node. If the error says not transform from x to /map then you need to make sure the transform is being published (use rosrun tf tf_echo! See [Debugging Tools](#debugging-tools))  

- **ssh Not Connecting**  
**Hardware/Software:** Hardware  
**Component:** WIFI 
**ROS Related:** No  
**Fix:** If ssh is not working first do a dumby check to make sure your IP and username are correct. Then try to `ping` the car IP. If it does not respond, then make sure the car is powered. If powered and still not pinging, then it likely is struggling to pickup wifi. Plug a HDMI cable into the TX2 on the car and use `ifconfig` to confirm wifi. If nothing still, then use the graphical interface to try to connect to wifi. Also, make sure your base computer is on the same network as the car.  