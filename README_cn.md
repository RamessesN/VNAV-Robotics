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
  <strong>基于 MIT VNAV (Visual Navigation for Autonomous Vehicles) 课程的机器人集成项目代码库。</strong>
</p>

<div align="center">

[English](README.md) | [中文](./README_cn.md)

</div>

---

## 📖 关于项目

本项目包含 **Robotics Integration Group Project I** 的所有实验代码与报告。内容涵盖了从 Linux/ROS 基础环境配置，到无人机（UAV）的运动规划、控制，以及视觉导航（VNAV）和 SLAM 系统的实现与评估。

主要基于 MIT 16.485 (Visual Navigation for Autonomous Vehicles) 2023 课程大纲进行开发。

## ⚙️ 环境依赖

本项目主要在 **Ubuntu 20.04** 环境下开发，依赖以下核心组件：

> **⚠️ 架构兼容性警告**：受限于模拟器的兼容性，**Lab 3 和 Lab 4 必须在 x86_64 架构的计算机上运行**。这些实验目前**不支持** ARM 架构系统（例如 Apple Silicon）。

+ **OS**: Ubuntu 20.04 LTS
+ **ROS**: ROS Noetic Ninjemys
+ **Languages**: C++ 14/17, Python 3.8+
+ **Build Tools**: CMake, Make, Catkin

## 📂 实验内容

| Lab | 主题 (Topic) | 描述 (Description) | 链接 |
| :---: | :--- | :--- | :---: |
| **Lab 1** | **Environment Configuration** | Linux, C++, Git, CMake 基础环境配置与工具链熟悉。 | [Notion](https://spurious-cornflower-507.notion.site/Lab-1-Linux-C-Git-e788685e06a84200ae587ffe64258c76) |
| **Lab 2** | **ROS Basics** | ROS 1 (Noetic) 的安装、节点通信、TF 坐标变换及基础使用。 | [Notion](https://spurious-cornflower-507.notion.site/Lab2-ROS-105e9f90e72480519605ed793e6662dc) |
| **Lab 3** | **3D Trajectory Following** | 无人机 3D 轨迹跟踪与几何控制算法实现。 | [MIT Lab3](https://vnav.mit.edu/labs_2023/lab3/exercises.html) |
| **Lab 4** | **Drone Control & Racing** | 高级无人机控制策略，穿越门竞速模拟。 | [MIT Lab4](https://vnav.mit.edu/labs_2023/lab4/exercises.html) |
| **Lab 5** | **Visual Tracking** | 视觉前端处理，特征提取与光流跟踪 (Visual Odometry Frontend)。 | [MIT Lab5](https://vnav.mit.edu/labs_2023/lab5/exercises.html) |
| **Lab 6** | **Visual Positioning** | 视觉后端优化，位姿估计与建图 (Visual Odometry Backend)。 | [MIT Lab6](https://vnav.mit.edu/labs_2023/lab6/exercises.html) |
| **Lab 7** | **Visual SLAM Comparison** | SLAM 系统性能评估 (ORB-SLAM3 vs Kimera vs LDSO)。 | [MIT Lab9](https://vnav.mit.edu/labs_2023/lab9/exercises.html) |

---

#### ⚠️ License: This project isn't open-source. See Details [LICENSE](LICENSE).