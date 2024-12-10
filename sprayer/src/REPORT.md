---
title: Agricultural Robotics System
author: Ming Shih Wang, Jiahao Li
date: Dec-2024
status: new
type: report
---

# Introduction
### Autonomous Turtlebot with Precision Watering System

This research project focuses on the development of an autonomous Turtlebot platform equipped with a precision water-dispensing system. The robot will navigate a structured indoor environment to identify and assess plants, determining both the necessity and appropriate quantity of water to apply. This system integrates advanced robotic navigation, computer vision, and AI-driven plant recognition to mimic the efficiency of precision agricultural technologies.

![Precision Agriculture Robot](./images/robot_img_pic.png)

## Project Goals

The Turtlebot will:

- **Autonomously navigate** within a defined space using systematic navigation algorithms.
- Use **computer vision techniques** (e.g., fiducial marker detection via OpenCV) for localization and orientation.
- Leverage **OpenAI-based models** for plant type identification, enabling tailored watering strategies based on plant-specific hydration needs.
- Activate a **water sprayer actuator** to deliver a precise volume of water upon successful identification and analysis of the plant.

## Inspiration

- **Technological inspiration:** This project draws inspiration from advanced agricultural robotics, such as sniper robots capable of treating vast crop fields with unparalleled efficiency while using 95% fewer chemicals.
  - [Sniper robot treats 500k plants per hour with 95% less chemicals](https://www.youtube.com/watch?v=sV0cR_Nhac0&ab_channel=Freethink)
- **Personal relevance:** The project also addresses the common issue of plant neglect, inspired by Jiahao's experience with underhydrated and dying plants.

By combining robotics, AI, and precision actuation, this initiative explores innovative solutions for sustainable plant care in agricultural and domestic contexts.

## Objective
    - Modify a battery powered sprayer so that it could be controlled using ROS publishers.
    - Develop a Turtlebot that:
        - Scans the room to locate the plants with fiducials. 
        - Uses computer vision (OpenCV) to detect objects of a specific color or real plants.
        - Activates a water sprayer when the targeted plant is identified.

# Challenges

## Hardware
    One of the biggest challenge for this project was to tackle hardware modifications as a team who has no experience in hardward work. 

    There are three main components that goes into making the sprayer remote controllable using ROS publisher.

    1. arduino uno
    <img src="./images/arduino.png" alt="Robot Image" width: 400px; margin: 50px">
    2. relay
    <img src="./images/relay.png" alt="Robot Image" width: 400px; margin: 50px">
    3. soldering
    <img src="./images/solder.png" alt="Robot Image" width: 400px; margin: 50px">

    [details](docs/faq/hardware/external_actuator_control.md)
## Software
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