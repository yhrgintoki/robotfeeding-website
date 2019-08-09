---
title: "Your Racecar's First ''Steps''"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: false
draft: false
active: true
duration: 15 - 30
difficulty: Beginner 
summary: Get your car started and teleop running 
weight: 1
---

### Introduction
This tutorial will get your car up and running teleoperation.

### Goal 
To get you driving your car around in teleop.

### Requirements
  - Complete the [hardware]() setup with your car
  - A desktop/laptop computer that can ssh into the car. 
  - Monitor (Wi-Fi Setup Only)
  - HDMI/Display Cable (Wi-Fi Setup Only)
  - USB Keyboard/Mouse (Wi-Fi Setup Only) 

## Booting Up
Make sure your sd card is plugged into your Jetson Nano and flush, this should be done from the hardware tutorial. If you have an AC to 5V power adapter you can plug that into the barrel connector on the USB port side of the Jetson Nano. If not, place a battery in the right side of the car and plug the connector into the matching connector going into the top half of the car. Plug in the right angled barrel connector into the Jetson Nano. The green light on the right side should turn on. The correct configuration is shown below.

{{< figure src="/tutorials/first_steps/plugged_in.jpg" caption="Everything plugged in correctly. Notice the barrel connector in the top right, the battery in the right slot, and the green light in the top left. " width="600">}}

By default your car creates it on network called `ROBOT AP`. On your separate computer, connect to this network and ssh into your car.

{{< highlight bash >}}
$ ssh robot@10.42.0.1 -X
{{< / highlight >}}

**Password:** prl_robot

If you experience any errors, make sure you are connected to `ROBOT AP` and you typed the above properly.

## Configure User & Hostname
Your connected! So let's change the password and hostname. This section is all done on the robot side. First let's change the password.

{{< highlight bash >}}
$ passwd 
{{< / highlight >}}

Enter prl_robot as the current password and then type your new password with the prompts. Now let's configure the hostname, it is currently set to goose. Open the hostname file.

{{< highlight bash >}}
$ sudo gedit /etc/hostname 
{{< / highlight >}}

You will be prompted to enter the password. Change goose to whatever you want to use and save & exit. Similarly open the `/etc/hosts` file.

{{< highlight bash >}}
$ sudo gedit /etc/hosts
{{< / highlight >}}

And replace any goose with your new hostname. Save & exit. Reboot the car, to see the effect. Note: you will have to ssh in again.

{{< highlight bash >}}
$ sudo reboot
{{< / highlight >}}

## Update Repos
Because mushr is always updating, you may have outdated code on your image. To fix this just enter `~/catkin_ws/src/mushr` and pull via vcstool.

{{< highlight bash >}}
$ cd ~/catkin_ws/src/mushr && vcs pull -n
{{< / highlight >}}

Note: If you edited mushr source code, this will be overwritten. You can instead pull repos individually

## Launch Teleop the Easy Way
Turn on the car and vesc by plugging their batteries in. Hold down the front button as the car is starting. After a minute or so the lidar should start spinning indicating teleop is running.

## Launch Teleop the Hard Way 
This is better for debugging your code. Turn on the car and vesc by plugging their batteries in. This will involve multiple windows. You can either make a bunch of windows on your laptop and ssh from each of them or use [tmux](https://www.hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/) from one window. Tmux is installed on the car image. First, ssh into the car. Then start `roscore`.

{{< highlight bash >}}
$ roscore
{{< / highlight >}}

In a new window, ssh into the car, then launch teleop.

{{< highlight bash >}}
$ roslaunch mushr_base teleop.launch
{{< / highlight >}}

You should see the lidar spinning and be able to steer with the controller, by holding down the left bumper, which acts as a safety, and moving the joysticks to accelerate and steer, as shown in the diagram below. 

{{< figure src="/tutorials/first_steps/teleop_controls.png" caption="" width="350">}}
## Setup Wi-Fi (Optional)
If you want to install additional software on the car, or be able to use the internet on your laptop while connected to the car then you will want to set the car up with a static IP instead of it's own network. To do this, you will need to plug a monitor, keyboard, and mouse into the car. The keyboard/mouse can go in any of the USB ports, Wi-Fi setup does not require the sensors to be connected.

Once hooked up, you should see the login screen or the desktop. Login using your new password. Then use the mouse to click on the Wi-Fi icon in the top right. Disconnect from `ROBOT_AP`. Then open the connection manager.

{{< highlight bash >}}
$ sudo nm-connection-editor
{{< / highlight >}}

Click on **Robot AP** then the **General** Tab. Uncheck **Automatically connect to this network when it is available**. Exit.

Open `/etc/modprobe.d/bcmdhd.conf` with sudo privileges.

{{< highlight bash >}}
$ sudo gedit /etc/modprobe.d/bcmdhd.conf
{{< / highlight >}}

If the second line (`options bcmdhd op_mode=2`) is not commented out then comment it out (`#options bcmdhd op_mode=2`), save the file and reboot the Nano. Otherwise, just close the file.

Connect to the network that the robot should have a static IP on. Do this by clicking on the opposing arrows symbol in the top right hand corner of the Ubuntu GUI and inspecting the Wi-Fi network list. If your robot is already connected to the correct network, then proceed to the next step. If it is already connected but to the wrong network, click *Disconnect* and then allow the Wi-Fi network list to refresh. Once the desired network appears in the Wi-Fi network list, click on it to connect. If the network does not appear in the Wi-Fi list, try using the **Connect to Hidden Wi-Fi Network\...** option.

Open the network connection editor again.

Highlight the connection that corresponds to the network the robot should be connected to and click **Edit\...**. The connection should have the same name as the network if the robot has never connected to this network before. If the robot has connected to this network before, it might have the same name but post-fixed with a number. If this is the case, choose the one with the highest post-fixed number.

Click on the **General** tab and check that **Automatically connect to this network when it is available** and **All users may connect to thisnetwork** are enabled.

{{< figure src="/tutorials/first_steps/general.png" caption="General tab with auto connect set" width="600">}}

Click on the **Wi-Fi** tab. Make sure that the **SSID:** field is set to be the name of the network that the robot will have a static IP on. For example, it could be *University of Washington*. Make sure that the **Mode:** dropdown is set to **Client**. In the **BSSID:** dropdown, choose the last option. In the **Device:** dropdown, choose the last option.

{{< figure src="/tutorials/first_steps/wifi.png" caption="Wi-Fi tab with BSSID and SSID set" width="600">}}

Click on the **Wi-Fi Security** tab. Check that the **Security:** dropdown is set to **None**.

Click on the **IPv4 Settings** tab. Set the following fields:  

-  **Method:** dropdown to be **Manual**.  
- Click **Add**. Under **Address**, enter:  
    - The Static IP: for example `172.16.77.Z`.  
    - Netmask: for example `255.255.255.0`.  
    - Gateway: for example `172.16.77.100`.  
    `Z` is usually the robot number on the car, but it's up to you!  
- **DNS servers:** field, enter `8.8.8.8`.  

{{< figure src="/tutorials/first_steps/ipv4.png" caption="IPv4 tab with an example IP setup" width="600">}}

Click the **Save** button and close the connection editor.

Edit `/home/robot/.bashrc` editing the line that sets the `ROS_IP` (near end of file). Alter this line so that the `ROS_IP` is set to the static ip, for example `172.16.77.Z`. 

{{< highlight bash >}}
$ gedit ~/.bashrc
{{< / highlight >}}

Alternatively if you know the network interface, you can use the command (should be `wlan0` interface on the Jetson):

{{< highlight bash >}}
export ROS_IP=$(ifconfig wlan0 | awk /inet\ /'{print $2}')
{{< / highlight >}}

Reboot the Nano.

After the Jetson reboots, verify that the robot has obtained the expected static ip.

{{< highlight bash >}}
$ ifconfig
{{< / highlight >}}

### Logging In

Once the Nano has fully booted, it will connect to the existing network at the specified static ip. You should then be able to ssh into it with the static IP you set earlier, in our case `172.16.77.Z`.

{{< highlight bash >}}
$ ssh robot@172.16.77.Z`
{{< / highlight >}}
