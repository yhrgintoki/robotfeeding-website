---
title: "Wireless Perception Module"
date: 2018-11-28T15:14:54+10:00
featured: true
draft: false
active: true
summary: A module which interfaces sensors and wirelessly transmits high-bandwidth data over WiFi
weight: 2
---

<font color=black>Introduction</font>

Robot manipulators glide through free space, constrained only by their own kinematics. Placing sensors like cameras or haptic transducers at their end-effectors enables egocentric interaction, but introduces a data transmission problem. A simple approach researchers often opt for is to fix wiring along the external shell of the robot. In practice, however, as the horizon of a robot’s task increases, so does the probability of the robot ensnaring itself in a tangle of its own wires.

We want our manipulators to be free of tethers, while remaining flexible to adding and subtracting sensors as research needs rapidly change. Many manipulators on the current market lack internal channels for transmitting high-bandwidth data, and running wires too close to internal conduits can subject high-bandwidth signals to noise from electromagnetic interference (crosstalk).

Our solution is a general-purpose module which interfaces sensors and wirelessly transmits high-bandwidth data over WiFi. We refer to it as the Wireless Perception Module or WiPer for short.

{{< figure src="/imgs/wpm1.png">}}

&nbsp;

## Software Architecture

The pipeline can be broken down into 5 major components. 1) Frame Acquisition: saving raw data from the sensor, 2) Encoding: using jpeg compression to reduce per frame file size and saving into packets, 3) WiFi: broadcasting packets on network, 4) Decoding: unpacking and decompressing the data, 5) Perception Tasks: completing a task using the data.

{{< figure src="/imgs/wpm2.png">}}

## Challenges and Limitations

Meeting this form factor’s requirements with mostly off-the-shelf components necessitated thoughtful design. As compute scaled, nominal power draw scaled. As nominal power draw scaled, thermal radiation scaled. As thermal radiation scaled, size of devices scaled due to cooling needs. Ultimately, we struck a nice balance with the Intel Joule and a Quarter-sized buck-converter. Along that vein, we had to be mindful of the amount of power our manipulator (in this case the Kinova Jaco 6DoF) was capable of sourcing. The Jaco can source up to 3A@24Vand the Joule needs roughly 1.5A@12V on bootup. Adding in sensors that draw power means we need to balance all three needs on the same bus: the manipulator’s motors, compute, and sensing.

The current software stack achieves an average of 29 FPS for RGB + D data, unaligned. When reconstructing a point cloud with RGBD data, it is necessary to temporally align the two signals. This, unfortunately, leads to drops in framerate for the reconstructed point cloud. This does not have a significant bearing on our current applications, but could be a limitation for future tasks. There is a workaround which involves a custom compression wrapper, but it excludes ROS and is currently undergoing development. We are also not taking full advantage of hardware acceleration for data compression on the Joule version — we expect work on this to lead to improvements in overall latency.

## Performance

__Choice of Network Matters.__ Seems obvious in retrospect, but something to pay attention to! Use a network without much traffic on it if you expect high framerates! Here’s a plot with uncompressed data from early on in our testing.

{{< figure src="/imgs/wpm3.png">}}

&nbsp;

__Compression.__ Our best performing methods tested on the Joule platform have been a combination of Jpeg for RGB and Zlib for Depth data (intraframe). We’ve experimented with interframe methods, but performance was less stable. This is something we are excited to use moving forward.

{{< figure src="/imgs/wpm4.png">}}

&nbsp;

__Framerate.__ The current average performance hovers around 29 FPS, depending on the complexity of the signal.

{{< figure src="/imgs/wpm5.png">}}

&nbsp;

__Latency.__ With compression, we achieved <50 ms which was more than sufficient for our current use cases.

{{< figure src="/imgs/wpm6.png">}}

&nbsp;

To see our system in action, check out the assistive food manipulation project!

