---
title: Agricultural Robotics System
author: Ming Shih Wang, Jiahao Li
date: Dec-2024
status: new
type: report
---

# Introduction
    This project involves building a Turtlebot that navigates a room and uses image processing to identify people wearing a specific color. When the target is identified, the Turtlebot activates an actuator to spray water, mimicking the behavior of a precision agriculture robot spraying crops.

    Inspo: [Sniper robot treats 500k plants per hour with 95% less chemicals](https://www.youtube.com/watch?v=sV0cR_Nhac0&ab_channel=Freethink)
    Personal Inspo: Jiahao's dead plants that never got watered

## Objective
    - Modify a battery powered sprayer so that it could be controlled using ROS publishers.
    - Develop a Turtlebot that:
        - Scans the room to locate the plants with fiducials. 
        - Uses computer vision (OpenCV) to detect objects of a specific color or real plants.
        - Activates a water sprayer when the targeted plant is identified.

# What was created (biggest section)
    Technical descriptions, illustrations
    Discussion of interesting algorithms, modules, techniques
    Guide on how to use the code written
    Tables listing names and one sentence purpose of each of these:
        Python source files
        Nodes created
        Topics and their messages


# Story of the project.
    How it unfolded, how the team worked together
    Your own assessment
    problems that were solved, pivots that had to be taken