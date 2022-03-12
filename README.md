1. Introduction 

   This repository contains source code database for Moorebot Scout, an AI-powered security mobile robot built upon Linux and ROS. Code name "Roller Eye”, Moorebot Scout is one of the most successful crowdfunded robotic projects. Equipped with many advanced sensors and AI algorithms, Scout is an ideal assistant for home monitoring with no blind spot. Its capabilities include object recognition, work with Amazon Alexa and Google Home, and monocular SLAM. For security reasons, Scout's communication layer is not part of the open source. Developers need to build their own communication. Developers can also build extension tools through the UART port on Scout's back.

2.Getting started  

  (1) .Requirements  
	The roller_eye needs to be compiled using the Ubuntu system. It is recommended to use Ubuntu18.04
	
  (2) .Build  
       Use the container linked below to compile in Ubuntu:
	 https://github.com/Pilot-Labs-Dev/binary.git

	After installing the container according to its instructions, compile according to the following steps:

	create ros workspace
	mkdir -p ros/src
	cd ros
	copy roller_eye source code
	cd src
	git clone this repo url or mv you roller_eye path .
	build roller_eye
	cd roller_eye
	./build.sh

  (3) .Sample  

      ①         How to get video stream  
       Subscribe to the topic "/CoreNode/h264" to get the frame message. Frame is a custom message, which is defined in the roller_eye/msg/frame.msg file.

      ②         How to get audio stream  
       Subscribe to "/CoreNode/aac" topic to get frame message, Frame is the same as video stream message.

      ③         How to get light sensor data  
       Subscribe to "/SensorNode/light" topic to get sensor_msgs::Illuminance message. The upper 16 bits of illumination indicate CH0 channel value, and the lower 16 bits 
      indicate ch1 channel value.
      ④         How to obtain TOF data  
       Subscribe to "/SensorNode/tof" topic to get sensor_msgs::Range message. The value of its range member is the distance value measured by TOF sensor, in meters

      ⑤         How to obtain IMU data  
       Subscribe to the topic "/SensorNode/imu" to get sensor_msgs::Imu message
  (4) .Vio 
  
        Vio source address:
	
        https://github.com/Pilot-Labs-Dev/vio.git
	
  (5) .ROOT permission
  
        Please use APP to obtain ROOT permission (Android App v1.4.1 / IOS App not yet supported)  
  
       1. Click firmware version 5 times, the ROOT option will appear at the bottom  

       2. Enter the password "123456"   
       
        ![image](https://github.com/Pilot-Labs-Dev/Scout-open-source/blob/main/png/bb616c1a9600e38da73e4ce31c9b8fb.jpg?raw=true)
        ![image](https://github.com/Pilot-Labs-Dev/Scout-open-source/blob/main/png/3c99e2d1bbb31cb91af77fdd0d39cc5.jpg?raw=true)


  (6) .Communication  
        e-mail: contactus@moorebot.com


