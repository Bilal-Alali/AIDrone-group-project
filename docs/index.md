# Autonomous Indoor Drone Exploration

Welcome to the documentation of **Group 2** for the Master Project *"Drones with Artificial Intelligence"* (Winter Semester 2026/27) at the Frankfurt University of Applied Sciences, supervised by Prof. Dr. Christian Baun.

---

## About the Project

The primary goal of this project is to develop, implement, and evaluate a drone-based solution for the **automatic exploration and mapping of indoor environments**. 

Since reliable GPS signals are unavailable inside buildings, traditional drone navigation fails. To solve this, our drone relies on a combination of alternative localization methods, including **Optical Flow, LiDAR distance measurements(, and visual odometry**). The system must safely navigate unknown corridors, avoid obstacles and create a 2D/3d occupancy map.

!!! tip "The Challenge"
    A major challenge of this project is the limited hardware capacity on board (Raspberry Pi Zero 2 with only 512 MB RAM). Therefore, we implemented a **3-layer architecture**: Real-time flight control is handled on the drone, while heavy computations like ROS 2 SLAM and path planning are offloaded to a Ground Control Station via a MAVLink Wi-Fi bridge (Raspberry Pi Zero 2).

---

## Objectives

Objectives & Milestones ... WIP

---

## Hardware Overview

WIP

---

## The Team

WIP