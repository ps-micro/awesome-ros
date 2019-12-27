# awesome-ros  
[![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/ps-micro/awesome-ros)    
This is a list of various resources for ROS and robotics. It's an attempt to gather useful material in one place for everybody who wants to learn more about the field.   
(including both **Chinese** and **English** materials)   

## contents:  
* [1.ROS](README.md#1ros)
* [2.ROS2](README.md#2ros2)
* [3.Robotics](README.md#3robotics)
* [4.Robots](README.md#4robots)
* [5.Vision](README.md#5vision)
* [6.Calibration](README.md#6calibration)
* [7.SLAM](README.md#7slam)
* [8.Localization](README.md#8localization)
* [9.Navigation](README.md#9navigation)
* [10.Control](README.md#10control)
* [11.Manipulator](README.md#11manipulator)
* [12.Machine Learning](README.md#12machine_learning)
* [13.Other Awesome Topics](README.md#13other_awesome_topics)

## 1.ROS
Websites:
* [ROS](https://www.ros.org) - provides libraries and tools to help software developers create robot applications
* [ROS Wiki](https://wiki.ros.org/) - ros packages and tutorials
* [ROSCon](https://roscon.ros.org) - the annual ROS developer conference
* [古月居](http://www.guyuehome.com/) - 中国ROS机器人开发社区
* [zhangrelay的专栏](https://blog.csdn.net/ZhangRelay) - 张瑞雷个人博客
* [易科机器人实验室ExBot](http://blog.exbot.net/) - 致力于机器人技术的创新与分享

Courses:
* [ROS入门21讲](https://www.bilibili.com/video/av59458869?from=search&seid=14130773534647455907) - 机器人开发之路扬帆起航

Books：
* [Mastering ROS for Robotics Programming](https://www.amazon.com/Mastering-ROS-Robotics-Programming-Operating/dp/1788478959), by Lentin Joseph
* [ROS By Example (Volume 1 and Volume 2)](http://wiki.ros.org/Books/ROSbyExample), by Patrick Goebel
* [Programming Robots with ROS: A Practical Introduction to the Robot Operating System](https://www.amazon.com/Programming-Robots-ROS-Practical-Introduction/dp/1449323898), by Morgan Quigley, Brian Gerkey & William D. Smart
* [Learning ROS for Robotics Programming](https://www.amazon.com/Learning-ROS-Robotics-Programming-Second-ebook/dp/B00YSIL6VM), by Aaron Martinez, Enrique Fernández
* [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/agitr-letter.pdf), by Jason M. O'Kane
* [ROS Robotics Projects: Make your robots see, sense, and interact with cool and engaging projects with Robotic Operating System](https://www.amazon.com/ROS-Robotics-Projects-interact-Operating/dp/1783554711), by Lentin Joseph 
* [Effective Robotics Programming with ROS](https://www.amazon.com/Effective-Robotics-Programming-ROS-Third-ebook/dp/B01H1JD6H6), by Anil Mahtani and Luis Sanchez
* [ROS机器人开发实践](https://item.jd.com/12377412.html), by 胡春旭

## 2.ROS2
* [ROS2 Documentation](https://index.ros.org/doc/ros2/) - ROS 2 Documentation
* [ros2/design](https://github.com/ros2/design) - Design documentation for ROS 2.0 effort.

## 3.Robotics
Courses:
* [Andrew Davison Robotics Course](http://www.doc.ic.ac.uk/~ajd/Robotics/index.html) - focuses on mobile robotics
* [ETH - Robotic Systems Lab](https://rsl.ethz.ch/education-students/lectures.html) 
* [斯坦福大学公开课——机器人学](https://www.bilibili.com/video/av4506104/) - 机器人学经典视频教程 
* [交通大学 —— 机器人学](https://www.bilibili.com/video/av18516816) 
* [开源机器人学学习指南](https://github.com/qqfly/how-to-learn-robotics) - 邱强博士整理的机器人学学习路径

Books：
* [Introduction to Robotics: Mechanics and Control](https://www.amazon.com/Introduction-Robotics-Mechanics-Control-3rd/dp/0201543613), by John J. Craig
* [Robotics: Modelling, Planning and Control](https://www.amazon.com/Robotics-Modelling-Planning-Textbooks-Processing/dp/1846286417), by Bruno Siciliano, Lorenzo Sciavicco, Luigi Villani, Giuseppe Oriolo
* [Probabilistic Robotics (Intelligent Robotics and Autonomous Agents series)](http://www.amazon.com/Probabilistic-Robotics-Intelligent-Autonomous-Agents/dp/0262201623/), by Sebastian Thrun, Wolfram Burgard, Dieter Fox
* [Robotics, Vision and Control: Fundamental Algorithms In MATLAB](https://www.amazon.com/Robotics-Vision-Control-Fundamental-Algorithms/dp/3319544128), by Peter Corke

## 4.Robots
* [ROS Robots](https://robots.ros.org/) - showcase robots using ROS

Mobile Robots:
* [PR2](http://wiki.ros.org/Robots/PR2) - a mobile manipulation platform built by Willow Garage
* [turtlebot3](http://wiki.ros.org/turtlebot3) - a new generation mobile robot that is modular, compact and customizable
* [Aldebaran Nao](http://wiki.ros.org/nao) - a commercially available humanoid robot built by Aldebaran.

Manipulator:
* [JACO](http://wiki.ros.org/jaco_ros) - a ROS interface for the Kinova Robotics JACO robotic manipulator arm
* [fanuc](http://wiki.ros.org/fanuc) - ROS-Industrial support for Fanuc manipulators
* [motoman](http://wiki.ros.org/motoman) - ROS-Industrial support for Yaskawa Motoman manipulators
* [universal_robots](http://wiki.ros.org/action/show/universal_robots?action=show&redirect=universal_robot) - ROS-Industrial support for Universal Robots manipulators
* [PROBOT Anno](http://wiki.ros.org/Robots/PROBOT_Anno) -  a ROS-based 6DoF 3D printed arm for research and automation

Component:
* [Shadow_Hand](http://wiki.ros.org/Robots/Shadow_Hand) -  a truly anthropomorphic approach to robot manipulation
* [Lego NXT](http://wiki.ros.org/Robots/NXT) - a modular robotics kit manufactured by Lego

## 5.Vision
* [find_object_2d](http://wiki.ros.org/find_object_2d) - simple Qt interface to try OpenCV implementations of SIFT, SURF, FAST, BRIEF and other feature detectors and descriptors. 
* [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) -  an open source framework built on top of TensorFlow that makes it easy to construct, train and deploy object detection models
* [object_recognition：](http://wiki.ros.org/object_recognition) - provide object recognition based on hough-transform clustering of SURF

## 6.Calibration
* [camera_calibration](http://wiki.ros.org/camera_calibration/) - easy calibration of monocular or stereo cameras using a checkerboard calibration target
* [visp_hand2eye_calibration](http://wiki.ros.org/visp_hand2eye_calibration) - estimates the camera position with respect to its effector using the ViSP library
* [easy_handeye](https://github.com/IFL-CAMP/easy_handeye) - wraps the hand-eye calibration routine from the ViSP library
* [handeye-calib-camodocal](https://github.com/jhu-lcsr/handeye_calib_camodocal) - generic robot hand-eye calibration.
* [robot_calibration](https://github.com/mikeferguson/robot_calibration) - generic robot kinematics calibration for ROS
* [kalibr](https://github.com/ethz-asl/kalibr) - camera and imu calibration for ROS

## 7.SLAM
* [gmapping](http://wiki.ros.org/gmapping) -  a ROS wrapper for OpenSlam's Gmapping
* [hector_slam](http://wiki.ros.org/hector_slam) - a SLAM approach that can be used without odometry
* [slam_karto](http://wiki.ros.org/slam_karto) - pulls in the Karto mapping library, and provides a ROS wrapper for using it
* [cartographer](http://wiki.ros.org/cartographer) - a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.
* [ohm_tsd_slam](http://wiki.ros.org/ohm_tsd_slam) - provides a 2D SLAM approach for laser scanners
* [slam_toolbox](http://wiki.ros.org/slam_toolbox) - provides a sped up improved slam karto with updated SDK and visualization and modification toolsets
* [orb_slam2_ros](http://wiki.ros.org/orb_slam2_ros) - ORB SLAM2 ros implementation

## 8.Localization
* [amcl](http://wiki.ros.org/amcl) - a probabilistic localization system for a robot moving in 2D
* [mrpt_localization](http://wiki.ros.org/mrpt_localization) - robot 2D self-localization using dynamic or static (MRPT or ROS) maps


## 9.Navigation
* [ROS Navigation Stack](http://wiki.ros.org/navigation/) -  takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base
* [move_base](http://wiki.ros.org/move_base/) - move a robot to desired positions using the navigation stack
* [ROS2 Navigation Packages](https://github.com/ros-planning/navigation2) - the control system that enables a robot to autonomously reach a goal state

## 10.Control
* [ros_control](http://wiki.ros.org/ros_control) - make controllers generic to all robots
* [ros_controllers](http://wiki.ros.org/ros_controllers) - library of ros controllers
* [ROS Control Tutorial](http://gazebosim.org/tutorials/?tut=ros_control) - setup simulated controllers to actuate the joints of your robot

## 11.Manipulator
MoveIt: 
* [ROS MoveIt](https://moveit.ros.org/) - Easy-to-use robotics manipulation platform for developing applications, evaluating designs, and building integrated products
* [MoveIt Tutorials](http://docs.ros.org/melodic/api/moveit_tutorials/html/index.html) - step by step examples for learning MoveIt
* [MoveIt Source Code API](https://moveit.ros.org/documentation/source-code-api/) - API document 

Kinematics:
* [IKFast: The Robot Kinematics Compiler](http://openrave.org/docs/0.8.2/openravepy/ikfast/) - solves robot inverse kinematics equations and generates optimized C++ files
* [Create_a_Fast_IK_Solution](http://wiki.ros.org/Industrial/Tutorials/Create_a_Fast_IK_Solution) - describes how to automatically create a fast, closed-form analytical kinematics solution
* [trac_ik](http://wiki.ros.org/trac_ik) - improved alternative Inverse Kinematics solver to the popular inverse Jacobian methods in KDL

Grasping:
* [agile_grasp](http://wiki.ros.org/agile_grasp) - finds antipodal grasps in point clouds
* [graspit](http://wiki.ros.org/graspit) - downloads and builds the GraspIt! simulator
* [moveit_simple_grasps](http://wiki.ros.org/moveit_simple_grasps) - a basic grasp generator for simple objects such as blocks or cylinders

## 12.Machine_Learning
* [openai_ros](http://wiki.ros.org/openai_ros) - a common structure for organizing everything you need to create your robot training from zero, requiring very little implementation
* [tensorflow_ros_cpp](http://wiki.ros.org/tensorflow_ros_cpp) - catkin-friendly C++ bindings for the tensorflow package
* [Tensorflow Object Detector with ROS](https://github.com/osrf/tensorflow_object_detector) - Tensorflow Object Detector with ROS


## 13.Other_Awesome_Topics
* [Awesome ROS2](https://fkromer.github.io/awesome-ros2/) - a curated list of ROS2 resources and libraries
* [Awesome Robotics](https://github.com/ahundt/awesome-robotics) - a curated list of links and software libraries that are useful for robots
