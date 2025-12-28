<div align="center"> 
  <h1> VNAV-Robotics </h1>
  <h2> Robotics Integration Group Project I </h2>
</div>

<div align="center">
  <img src="https://img.shields.io/badge/OUC%20x%20MIT-green" alt="School">
  <img src="https://img.shields.io/badge/ROS-Noetic-blue" alt="ROS">
  <img src="https://img.shields.io/badge/Language-C++%20|%20Python-orange" alt="Language">
  <a href="./LICENSE"><img src="https://img.shields.io/badge/License-Proprietary-lightgrey" alt="License"></a>
</div>

<br>

<p align="center">
  <strong>Robotics integration project repository based on the MIT VNAV (Visual Navigation for Autonomous Vehicles) course.</strong>
</p>

<div align="center">

[English](README.md) | [中文](./README_cn.md)

</div>

---

## 📖 About the Project

This repository contains all experimental code and reports for **Robotics Integration Group Project I**. The content ranges from basic Linux/ROS environment configuration to UAV motion planning and control, as well as the implementation and evaluation of Visual Navigation (VNAV) and SLAM systems.

It is primarily developed based on the **MIT 16.485 (Visual Navigation for Autonomous Vehicles) 2023** curriculum.

## ⚙️ Prerequisites

This project is primarily developed in an **Ubuntu 20.04** environment and relies on the following core components:

> **⚠️ Architecture Warning**: Due to simulator compatibility issues, **Lab 3 and Lab 4 strictly require an x86_64 architecture**. These labs are currently **NOT** compatible with ARM-based systems (e.g., Apple Silicon).

+ **OS**: Ubuntu 20.04 LTS
+ **ROS**: ROS Noetic Ninjemys
+ **Languages**: C++ 14/17, Python 3.8+
+ **Build Tools**: CMake, Make, Catkin

## 📂 Labs Overview

| Lab | Topic | Description | Link |
| :---: | :--- | :--- | :---: |
| **Lab 1** | **Environment Configuration** | Basic environment configuration and toolchain familiarization for Linux, C++, Git, and CMake. | [Notion](https://spurious-cornflower-507.notion.site/Lab-1-Linux-C-Git-e788685e06a84200ae587ffe64258c76) |
| **Lab 2** | **ROS Basics** | Installation of ROS 1 (Noetic), node communication, TF coordinate transforms, and basic usage. | [Notion](https://spurious-cornflower-507.notion.site/Lab2-ROS-105e9f90e72480519605ed793e6662dc) |
| **Lab 3** | **3D Trajectory Following** | Implementation of UAV 3D trajectory following and geometric control algorithms. | [MIT Lab3](https://vnav.mit.edu/labs_2023/lab3/exercises.html) |
| **Lab 4** | **Drone Control & Racing** | Advanced UAV control strategies and simulation of drone racing through gates. | [MIT Lab4](https://vnav.mit.edu/labs_2023/lab4/exercises.html) |
| **Lab 5** | **Visual Tracking** | Visual frontend processing, feature extraction, and optical flow tracking (Visual Odometry Frontend). | [MIT Lab5](https://vnav.mit.edu/labs_2023/lab5/exercises.html) |
| **Lab 6** | **Visual Positioning** | Visual backend optimization, pose estimation, and mapping (Visual Odometry Backend). | [MIT Lab6](https://vnav.mit.edu/labs_2023/lab6/exercises.html) |
| **Lab 7** | **Visual SLAM Comparison** | Performance evaluation of SLAM systems (ORB-SLAM3 vs. Kimera vs. LDSO). | [MIT Lab9](https://vnav.mit.edu/labs_2023/lab9/exercises.html) |

---

#### ⚠️ License: This project isn't open-source. See Details [LICENSE](LICENSE).