# WatDig: Autonomous Navigation Robot

## Introduction
Welcome to the repository for WatDig, an innovative autonomous navigation robot designed for The Boring Company's Not-a-Boring Competition. WatDig navigates complex underground environments autonomously, without GPS, leveraging the robust and flexible ROS2 Humble architecture.

## Features
- **Autonomous Navigation**: Utilizes advanced algorithms for navigating through subterranean environments.
- **ROS2 Humble Architecture**: Ensures modularity and integration ease, making it ideal for complex robotic applications.
- **Sensor Fusion**: Combines data from various sensors for precise environmental interaction.
- **Real-Time Mapping**: Generates on-the-fly maps of unknown terrains, aiding in efficient pathfinding and obstacle avoidance.

## Prerequisites
Before installation, ensure you have:
- ROS2 Humble installed
- Python 3.7 or newer
- Necessary hardware drivers (refer to `hardware_setup.md`)

## Installation
To install WatDig, follow these steps:
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/your-repository/watdig.git
2. **Navigate to the Directory**
   ```bash
   cd watdig
3. **Install Dependencies**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y

## Usage
Launch watdig using the command
```bash
ros2 launch watdig_navigation start.launch.py
```
## Configuration
- **Sensors**: Configure **'conifg/sensors.yaml'**
- **Navigation Parameters**: Adjust in **'config/navigation.yaml'**

