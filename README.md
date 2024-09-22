# Autonomous Mobile Robot Project

This project involves the development of an autonomous mobile robot capable of self-localization, object detection, and performing wall-following maneuvers in real-time. The robot demonstrates advanced control and localization techniques to navigate its environment and execute tasks independently.

## Key Features

- **Autonomous Self-Localization**: The robot utilizes HTC VIVE for accurate localization to navigate towards predefined target locations.
- **Object Detection**: The robot can detect objects within its immediate environment using infrared sensors.
- **Wall-Following**: PID control is implemented to ensure precise and stable wall-following behavior.
- **Competition-Ready**: The robot participated in the Grand Theft Autonomous competition, where it successfully detected and retrieved a trophy based on infrared signal tracking.

## Project Contributions

- **PID Control Implementation**: Developed a PID controller for optimal wall-following performance, enabling the robot to navigate corridors and avoid obstacles with precision.
- **Localization Integration**: Utilized HTC VIVE for real-time localization, allowing the robot to determine its exact position and move accurately towards the target location.
- **C Programming**: Wrote C code for implementing control algorithms, sensor integration, and overall robot behavior.
- **Competition Participation**: Contributed to the robot's success in the Grand Theft Autonomous competition, where it autonomously identified and retrieved a trophy emitting an infrared frequency from a 2-meter distance, returning it to the base.

## Getting Started

### Prerequisites

- ESP32 Microcontroller 
- C/C++ compiler
- HTC VIVE system for localization
- Infrared sensors and Ultrasonic sensors for object detection

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/your-username/autonomous-robot-project.git
