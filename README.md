# Robot Maze Solver

This project is a mobile robot maze solver built using ROS 2, Gazebo, TensorFlow, and Deep Q Learning. The goal of the project is to train a robot to navigate through maze environments using sensor data and reinforcement learning.

The robot uses distance sensor readings to understand its surroundings and decide how to move through the maze. A Deep Q Network is used to choose actions based on the robot’s current state, allowing it to learn better navigation behavior over time.

## Technologies Used

- ROS 2
- Gazebo
- Python
- TensorFlow/Keras
- Deep Q Learning
- Reinforcement Learning
- Distance sensors

## How It Works

The project is organized around separate ROS 2 nodes that handle different parts of the robot system. One part of the system reads sensor data, another controls the robot’s movement, and another runs the Deep Q Learning model that decides what action the robot should take.

The robot is tested in a Gazebo simulation environment, where it can move through mazes and learn from rewards based on its actions.

## Project Note

This project was developed using a specific ROS 2 and Gazebo setup with custom filenames, launch files, and simulation files. It may require adjustments before running on another machine.
