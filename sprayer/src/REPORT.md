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

The objective of this project is to explore the integration of robotics, artificial intelligence, and precision actuation to solve real-world problems related to plant care and resource management. By leveraging autonomous navigation, image recognition, and intelligent decision-making systems, the project aims to:

- Demonstrate the feasibility of using robotics for sustainable, resource-efficient plant care.
- Develop a scalable framework for integrating AI-driven plant identification into robotic systems.
- Investigate and optimize strategies for precision watering to minimize waste while meeting plant-specific needs.
- Provide insights into the potential of robotic platforms in precision agriculture and domestic gardening.

This project serves as a proof of concept for innovative approaches to automated plant care, with broader implications for agricultural and environmental sustainability.


# Challenges

## Color/Plant Detection Accuracy: 
Ensuring that the Turtlebot reliably detects the target plant under varying lighting conditions (shadows or brightness changes) is crucial. (TALK ABOUT THE FAILED USE OF YOLO AND TRANSITION TO OPEN API )

## Water Sprayer Signaling System:
Create a signaling channel for the turtlebot to control the sprayer through ROS. This requires hardware level development. 

One of the biggest challenge for this project was to tackle hardware modifications as a team who has no experience in hardward work. 

There are three main components that goes into making the sprayer remote controllable using ROS publisher.

1. arduino uno

The arduino is responsible for receiving messages from rasberry pi and controlling the relay.
<img src="./images/arduino.png" alt="arduino" width: "400px">

2. rasberry pi

The rasberry pi is where the ROS subscriber is run. It listens to published messages and passes it down to arduino uno.


3. relay

The relay is responsible for controlling the open and close of the circuit loop which triggers the power of the sprayer.

There are 6 ports on the relay. Each of them except NC is required for our setup 

IN: Connects to the Arduino's GPIO pin (e.g., pin 7), this port handles recieving commands from arduino uno. 
DC+: Connects to Arduino 5V, along with DC- this port provides the power to trigger the relay.
DC-: Connects to Arduino GND.
COM: Connects to the live wire or signal going to the load (e.g., a light bulb or motor).
NO: The load should be OFF by default and turn ON when the relay is activated.
NC: Since we want the relay to be OFF by default, this port is not necessary. 

<img src="./images/relay.png" alt="relay" width: "400px">




<img src="./images/solder.png" alt="solder" width: "400px">



[details](docs/faq/hardware/external_actuator_control.md)

## Water Spraying Mechanism Control: 
Verifying that the water spraying actuator can be accurately triggered at the right time and location. The challenge is to time the activation properly and test its range to ensure it only targets specific areas.

## Autonomous Navigation: 
Testing the Turtlebot’s ability to navigate and avoid obstacles reliably. 
(Fiducial nav / user gets to decide which to navigate to)


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