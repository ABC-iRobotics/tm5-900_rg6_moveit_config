# TM5-900 + RG6 MoveIt 2 configuration

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![repo size](https://img.shields.io/github/repo-size/ABC-iRobotics/tm5-900_rg6_moveit_config)
![GitHub Repo stars](https://img.shields.io/github/stars/ABC-iRobotics/tm5-900_rg6_moveit_config)
![GitHub forks](https://img.shields.io/github/forks/ABC-iRobotics/tm5-900_rg6_moveit_config)

## Introduction
MoveIt 2 configuration for a Techman TM5-900 equiped with an OnRobot RG6 gripper.

This package does not rely on the ```tm5-900``` nor the ```onrobot_rg_description``` packages.

The package has a USD example scene description for Isaac Sim usage.

## Features

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&category=Laptop&vendor=Dell&vendor=HP&vendor=Lenovo&release=22.04+LTS)
- [ROS 2 Humble (Python3)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- ROS 2 description of a TM5-900 equiped with an OnRobot RG6 gripper
- USD descriptions both for the [TM5-900](https://github.com/TM-Vision/tm-digital-robot-publish/blob/main/exts/tmrobot.digital_robot/tmrobot/digital_robot/assets/robot_series/tm5_900.usd) and the OnRobot RG6
- Isaac Sim scene using the *Simple Room* asset

## Prerequisites

- [MoveIt 2](https://moveit.picknik.ai/main/index.html)

## Setup Guide

Navigate into your ROS 2 workspace and copy this repo to your source folder
```
cd src && git clone https://github.com/ABC-iRobotics/tm5-900_rg6_moveit_config.git && cd ..
```

Build the package (and installing python prerequisites)
```
colcon build --packages-select tm5-900_rg6_moveit_config
```

## Usage

After installation the package can be used as any other MoveIt configuration in a ROS 2 launch file.

## Disclaimer

The physical accuracy of the models heavily depend on the assumed stiffness and damping of the joints. These values are choosen heuristically until the most realistic behaviours are reached.

There is always room for improvement.

## Author

[Makány András](https://github.com/andras-makany) - Graduate student at Obuda University

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).