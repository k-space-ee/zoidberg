# Woop-woop-woop

This is the source code repository of software stack used by the robot Zoidberg on Robotex 2018 basketball 1vs1 competition.

<img src="doc/woopwoop.jpg"/>

## Software

The source code for the robot can be found in [khajiit] directory.
It's written in Python, it makes use of OpenCV bindings for image recognition and ROS libraries to implement message passing between processes. We use 8pcs PS3 eye cameras stacked together along the narrow edges, thus we get 3840x640 (4k!!!) resolution. Due to USB 2.0 bandwidth limits we are capped at 30fps for the maximum resolution. Image recognition thread latency is 16-17ms on Skylake i7 BRIX which drops to about 20ms if visualization web interface is opened. There is a web interface for debugging output, logs are sent using websocket to the browser and arbitrary gamepads can be used to control the robot using HTML5 gamepad API.

Ansible scripts for deploying the software and installing/compiling the dependencies are included in [zoid-setup]

Thanks to [Zubax Robotics](https://zubax.com/) for supplying us with a reliable ESC-s for running the thrower brushless motor!
