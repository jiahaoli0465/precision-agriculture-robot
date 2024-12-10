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

### Plant Detection

Ensuring that the Turtlebot reliably detects the target plant under varying lighting conditions (e.g., shadows or brightness changes) is a critical component of this project. The initial approach involved integrating a YOLOv5 model for real-time plant detection, leveraging resources such as the [Plant Leaf Detection and Classification model](https://huggingface.co/foduucom/plant-leaf-detection-and-classification). However, this approach revealed several limitations:

- **Accuracy**: The model struggled with detecting plants under diverse lighting conditions and varying angles.
- **Versatility**: It lacked adaptability to different environmental settings, which limited its practical application.
- **Computational Load**: The YOLOv5 model was computationally intensive, making it less suitable for real-time deployment on a resource-constrained platform like the Turtlebot.

To address these challenges, the project transitioned to using the **OpenAI GPT-4o-mini model** with advanced vision capabilities. This model demonstrated significantly improved performance by:

- Providing **consistent detection accuracy**, achieving nearly 100% reliability across diverse conditions.
- Offering **restricted outputs** tailored to specific plant identification, enhancing precision.
- Operating with **greater computational efficiency**, making it a more practical option since computation was offloaded to an external API.

![Precision Agriculture Robot](./images/plant_compare.png)

#### Implementation Code: Plant Detection with OpenAI GPT-4o-mini

The `Detector` class leverages the GPT-4o-mini model to identify plant types or confirm the presence of a plant in an image. Below is the implementation:

```python
import base64
from openai import OpenAI
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

# Retrieve the OpenAI API key from the environment variables
open_api_key = os.getenv("OPEN_API_KEY")

class Detector:
    def __init__(self):
        self.client = OpenAI(api_key=open_api_key)  # Initialize OpenAI client
        self.MAX_RETRIES = 10  # Maximum retries for API requests
        self.plant_types = ['Cactus', 'Basil', 'Thyme', 'Parsley', 'Gatorade']  # Recognized plant types

    def detect_plant(self, image):
        """
        Detects the type of plant or Gatorade in an image.

        Args:
            image (str): Base64 encoded string of the image.

        Returns:
            tuple: (bool, str) indicating whether detection was successful and the identified plant type.
        """
        for i in range(self.MAX_RETRIES):
            try:
                response = self.client.chat.completions.create(
                    model="gpt-4o-mini",
                    messages=[
                        {
                            "role": "user",
                            "content": [
                                {"type": "text", "text": "Output the plant type or Gatorade and only the plant type in one word: 'Cactus', 'Basil', 'Thyme', 'Parsley', or 'Gatorade' if the image's object of interest contains the plant or Gatorade"},
                                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image}"}}
                            ],
                        }
                    ],
                )
                res = response.choices[0].message.content.strip()

                if res in self.plant_types:
                    return True, res
                else:
                    return False, None
            except Exception as e:
                print(f"Failed attempt {i}: {e}")

    def is_plant(self, image):
        """
        Determines whether an image contains any plant.

        Args:
            image (str): Base64 encoded string of the image.

        Returns:
            bool: True if the image contains a plant, False otherwise.
        """
        for i in range(self.MAX_RETRIES):
            try:
                response = self.client.chat.completions.create(
                    model="gpt-4o-mini",
                    messages=[
                        {
                            "role": "user",
                            "content": [
                                {"type": "text", "text": "Output in one word 'true' or 'false' if the image contains any plant"},
                                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image}"}}
                            ],
                        }
                    ],
                )
                res = response.choices[0].message.content.strip().lower()

                return res == "true"
            except Exception as e:
                print(f"Failed attempt {i}: {e}")

```

#### Explanation of Key Components

##### API Initialization:
The OpenAI client is initialized using an API key loaded from environment variables via the `dotenv` library. This ensures secure handling of sensitive information.

##### Plant Type Detection:
The `detect_plant` method sends a base64-encoded image to the OpenAI GPT-4o-mini model, instructing it to identify the object of interest from a predefined list of plant types (e.g., 'Cactus', 'Basil', etc.) or Gatorade.

##### Plant Presence Detection:
The `is_plant` method determines whether the image contains any plant, returning a boolean value.

##### Retry Mechanism:
Both methods implement a retry mechanism to handle potential API request failures, ensuring robust performance in real-world applications.

This refinement in plant detection methodology highlights the importance of balancing model accuracy, versatility, and computational feasibility in robotics applications. The GPT-4o-mini model proved to be a game-changer, ensuring robust and reliable plant identification for the Turtlebot's precision watering tasks. It can also detect a variety of objects outside of the plant constraints, including a Gatorade bottle.

Our project culture is to identify hard challenges and break them down with the simplest solutions. We avoided training a large detection model by integrating an advanced vision model.


## Water Sprayer Signaling System:
Create a signaling channel for the turtlebot to control the sprayer through ROS. This requires hardware level development. 

One of the biggest challenge for this project was to tackle hardware modifications as a team who has no experience in hardward work. 

### Components
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

### Soldering
We also learned soldering in part of this modification. There are two different approaches we can take. 

1. Directely tap into the trigger portion of the sprayer and 



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

pivoting from using a claw to 