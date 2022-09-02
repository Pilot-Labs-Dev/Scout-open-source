# Introduction

This repository contains source code database for Moorebot Scout, an AI-powered security mobile robot built upon Linux and ROS. Code name "Roller Eye”, Moorebot Scout is one of the most successful crowdfunded robotic projects. Equipped with many advanced sensors and AI algorithms, Scout is an ideal assistant for home monitoring with no blind spot. Its capabilities include object recognition, work with Amazon Alexa and Google Home, and monocular SLAM. For security reasons, Scout's communication layer is not part of the open source. Developers need to build their own communication. Developers can also build extension tools through the UART port on Scout's back.

# Getting started

## Requirements
The roller_eye needs to be compiled using the Ubuntu system. It is recommended to use Ubunt 18.04.
	
## Build
Use the container linked below to compile in Ubuntu: https://github.com/Pilot-Labs-Dev/binary.git

After installing the container according to its instructions, compile according to the following steps:

```
# Create ros workspace
mkdir -p ros/src
cd ros

# Copy roller_eye source code
cd src
git clone this repo url or mv you roller_eye path .

# Build roller_eye
cd roller_eye
./build.sh
```

# Samples

## How to get video stream
Subscribe to the topic `/CoreNode/h264` to get the frame message. Frame is a custom message, which is defined in the `roller_eye/msg/frame.msg` file.

## How to get audio stream
Subscribe to `/CoreNode/aac` topic to get frame message, Frame is the same as video stream message.

## How to get light sensor data
Subscribe to `/SensorNode/light` topic to get `sensor_msgs::Illuminance` message. The upper 16 bits of illumination indicate CH0 channel value, and the lower 16 bits indicate ch1 channel value.

## How to obtain TOF data
Subscribe to `/SensorNode/tof` topic to get `sensor_msgs::Range message`. The value of its range member is the distance value measured by TOF sensor, in meters.

## How to obtain IMU data
Subscribe to the topic `/SensorNode/imu` to get `sensor_msgs::Imu` message
       
## How to run Python code
Create a python fille like `example.py` and define your function, for example, the following defines a start() function;

```python
def start():
	rollereye.timerStart()
	rollereye.set_rotationSpeed(100)
	rollereye.set_translationSpeed(0.3)
	while rollereye.getTimerTime() <= 12000:
		rollereye.set_translate_rotate(2,270)
```

Add the import library above the function
```python
# -*- coding: utf-8 -*-
import sys
sys.path.append("/usr/local/lib")
from rollereye import *
rollereye.start()
```
Add the main function below the function
```python
if __name__ == '__main__':
		try:
			start()
		except Exception as e:
			rollereye.handle_exception(e.__class__.__name__ + ': ' + e.message)
		rollereye.stop()
```

A complete python script is as follows

```python
# -*- coding: utf-8 -*-
import sys
sys.path.append("/usr/local/lib")
from rollereye import *
rollereye.start()

def start():
	rollereye.timerStart()
	rollereye.set_rotationSpeed(100)
	rollereye.set_translationSpeed(0.3)
	while rollereye.getTimerTime() <= 12000:
		rollereye.set_translate_rotate(2,270)

if __name__ == '__main__':
		try:
			start()
		except Exception as e:
			rollereye.handle_exception(e.__class__.__name__ + ': ' + e.message)
		rollereye.stop()
```

Run the python script

```
python example.py
```

# Vio
Vio source address: https://github.com/Pilot-Labs-Dev/vio.git
	
# ROOT permission
  
Please use APP to obtain ROOT permission (Android App v1.4.1 / IOS App not yet supported)

1. Click firmware version 5 times, the ROOT option will appear at the bottom
2. Enter the password `123456`
       
![image](https://github.com/Pilot-Labs-Dev/Scout-open-source/blob/main/png/bb616c1a9600e38da73e4ce31c9b8fb.jpg?raw=true)


![image](https://github.com/Pilot-Labs-Dev/Scout-open-source/blob/main/png/3c99e2d1bbb31cb91af77fdd0d39cc5.jpg?raw=true)


# Communication
E-mail: contactus@moorebot.com
