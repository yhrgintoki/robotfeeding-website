---
title: "Quickstart"
date: 2018-11-28T15:14:54+10:00
image: "/services/default.png"
featured: true
draft: false
active: true
duration: 30 
difficulty: Beginner 
summary: Run the MuSHR platform on your machine! 
weight: 1
---

### Introduction
This tutorial will get you started with mushr in simulation!

### Goal 
To get the simulator running on your machine so that you can begin hacking immediately!

### Requirements
A linux machine (OSX and Windows support coming soon!). If you don't run linux natively then get a linux VM: [OSX](https://www.instructables.com/id/How-to-Create-an-Ubuntu-Virtual-Machine-with-Virtu/), [Windows](https://itsfoss.com/install-linux-in-virtualbox/).

## Setup
First we need to make sure you have a few dependencies installed. To make it easier, we have created a Docker container with the mushr stack and sim running inside it. Here is what you need:

- [docker](https://docs.docker.com/v17.12/install/)
- [docker-compose](https://docs.docker.com/compose/install/)
- git
- A github account. You can signup for one [here](https://github.com/join?source=header-home)
- An nvidia graphics card

{{< highlight bash >}}
$ sudo apt install git-all
{{< / highlight >}}

Once you have these, you're good to go!

## Install Sim
Now that we have the dependencies, lets get started! Note, we are assuming you have set up Docker to not need sudo with every call. You can set that up by following [these](https://docs.docker.com/install/linux/linux-postinstall/) steps. Now, open a terminal (CTRL + ALT + T) check if Docker is running:

{{< highlight bash >}}
$ docker run hello-world
{{< / highlight >}}

If you get a error then run:

{{< highlight bash >}}
$ systemctl start docker
{{< / highlight >}}

Let's clone the sim repo:

{{< highlight bash >}}
$ git clone https://github.com/prl-mushr/mushr_sim && cd mushr_sim/docker/
{{< / highlight >}}

Alright, so you are in the Docker directory of the sim. There are two configuration changes we need to make. First let's change the uid/gid of your Docker user to match the current user. This is required for GUI apps like rviz to connect. Check your UID/GID with the following command

{{< highlight bash >}}
$ id -u $USER
{{< / highlight >}}

for UID and

{{< highlight bash >}}
$ id -g $USER
{{< / highlight >}}

for GID. They usually are the same. Now that you know these values use your favorite text editor to change line 4 in Dockerfile. We will use gedit here

{{< highlight bash >}}
$ gedit Dockerfile
{{< / highlight >}}

And change line 4's UID/GID to match yours. The last configuration we need to do is make sure your nvidia driver matches the one the sim is looking for. This is required because OpenGL is needed for rviz. To see which nvidia driver your computer has run

{{< highlight bash >}}
$ ls /usr/lib/ | grep nvidia
{{< / highlight >}}

The default is `nvidia-390`. If your's is not that then open `.env` and change the number to match.

{{< highlight bash >}}
$ gedit .env
{{< / highlight >}}

That's it for setting up! We're done the hard part :)

## Run Sim
To start the sim run

{{< highlight bash >}}
$ docker-compose up -d
{{< / highlight >}}

You should see a small gray window pop up and rviz with the car model (see below). We've just created a container that you can see if you run 

{{< highlight bash >}}
$ docker ps
{{< / highlight >}}

{{< figure src="/tutorials/quickstart/rviz_docker.png" caption="The rviz window that should pop up after running `docker-compose`" width="800">}}

Give the car an initial position by clicking `2D Pose Estimate` in rviz and clicking and dragging in the main window. Now you can click on the small gray window and use the WSAD keys to drive the car around!

## Going Further
So driving the car around is fun, but what if you want to program it? So while everything else is running, in a new terminal run

{{< highlight bash >}}
$ docker exec -it CONTAINER-ID bash
{{< / highlight >}}

You can get the `CONTAINER-ID` from 

{{< highlight bash >}}
$ docker ps
{{< / highlight >}}

This gets you a bash shell inside the container. You will find all the sim code in `~/catkin_ws/src/mushr_sim`. The user developer won't have root privileges so you can't install software. To enter the container as root run

{{< highlight bash >}}
$ docker exec -it -u 0 CONTAINER-ID bash
{{< / highlight >}}

You can also separate launching the container from launching the sim. To do that edit line 16 in `docker-compose.yml` to `entrypoint: bash`. Then run 

{{< highlight bash >}}
$ docker-compose build
$ docker-compose up -d
{{< / highlight >}}

Then enter the container using the commands mentioned previously and source your workspace.

{{< highlight bash >}}
$ . ~/.bashrc
{{< / highlight >}}

 You can start the sim with

{{< highlight bash >}}
$ roslaunch mushr_sim teleop.launch
{{< / highlight >}}

And start rviz with

{{< highlight bash >}}
$ rviz
{{< / highlight >}}

To learn about programming the car continue to the [Intro to ROS Tutorial](/tutorials/intro-to-ros)
