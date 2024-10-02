# **Agri-Bot for Greenhouse Automation**
This project focuses on an agricultural robot (Agri-Bot) that navigates a simulated greenhouse environment to detect, pick, and place ripe tomatoes. The bot uses Intel RealSense D435i for object detection and depth estimation, with image thresholding for identifying ripe tomatoes. Additionally, Lidar is used for obstacle avoidance and navigation.

## Table of Contents
- [Project Overview](#project-overview)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Images](#images)
- [Video Demo](#video-demo)
- [Contributing](#contributing)

## Project Overview

The Agri-Bot autonomously moves within a greenhouse simulation to identify ripe tomatoes based on color detection. It leverages the Intel RealSense D435i camera for depth sensing and object detection. The bot uses Lidar for obstacle avoidance, ensuring smooth navigation while avoiding collisions with objects in the greenhouse.

## Features

- **Tomato Detection**: Identifies ripe tomatoes using image thresholding based on red color.
- **Depth Estimation**: Uses Intel RealSense D435i for accurate depth sensing and positioning.
- **Lidar-Based Navigation**: Avoids obstacles during movement using  RPLidar data.
- **Autonomous Pick and Place**: Capable of identifying ripe tomatoes and performing pick-and-place tasks.
- **Simulated Environment**: A fully simulated greenhouse environment for testing.

## Installation

### Prerequisites
Ensure the following dependencies are installed:

- ROS Noetic
- `rosdep`
  ```
  sudo rosdep init
  rosdep update
  cd ROS-agri-robot-harvester
  rosdep install --from-paths src --ignore-src -r -y
  ```

- `moveit` package
  ```
  sudo apt-get install ros-noetic-moveit-resources-prbt-moveit-config
  sudo apt-get install ros-noetic-moveit
  ```
 

### Steps
1. Clone the repository:
```
git clone https://github.com/S-Sidharthan/ROS-agri-robot-harvester.git
```
2. Build the project:
```
cd ROS-agri-robot-harvester
catkin_make
```
3. Source the workspace:
```
source devel/setup.bash
```
## Usage
Start Agri-Bot
To launch the Agri-Bot in the greenhouse environment:
#### Wall Following 
```
roslaunch ebot_gazebo wall_follow.launch
```
#### Pick and Place
```
roslaunch ebot_gazebo pick_and_place.launch
```
#### Detect Ripe tomatoes
```
roslaunch ebot_gazebo nav_pick_and_place.launch
```

## Images
Figure 1: Agri-Bot in gazebo environment.
![Screenshot from 2024-10-02 20-00-12](https://github.com/user-attachments/assets/503a1e2b-93fa-4526-98a9-e424348dffa9)


Figure 2: Ripe Tomatoes detection.
![Screenshot from 2024-10-02 20-50-21](https://github.com/user-attachments/assets/a97e730d-b417-4598-ba2d-0f248816f2c1)

## Video Demo

Watch a demo of the robot in action on YouTube:

[https://www.youtube.com/watch?v=iV6wcWm2Lak](https://www.youtube.com/watch?v=v5cbOS9KxQ8&ab_channel=Sidharthan)  

[https://www.youtube.com/watch?v=Go9n7JF8ANU&ab](https://www.youtube.com/watch?v=Go9n7JF8ANU&ab_channel=Sidharthan)





## Contributing
Contributions are welcome! Open issues or submit pull requests to improve the project.
