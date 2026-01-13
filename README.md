# webot-command-based-object-finder-robo
A Webots simulated robot that finds objects based on natural language commands using NLP and computer vision.

## Author
Jahangeer Khan

## Project Overview
This project implements a simulated mobile robot in Webots that can locate and move towards objects based on simple natural language commands.

The robot combines:
- Keyword-based Natural Language Processing (NLP)
- Computer Vision (color and shape detection)
- Reactive robot control

All experiments are conducted entirely in simulation using Webots.

---

## Example Commands
The robot can understand commands such as:
- "Find the red ball"
- "Go to the green cylinder"
- "Find the blue cone"

The command is parsed to extract:
- Object color
- Object type (shape)

---

## System Components

### 1. Natural Language Processing
The robot uses simple keyword-based NLP to identify the target color and object type from text commands.

### 2. Computer Vision
A camera sensor is used to detect objects based on:
- HSV color thresholding
- Contour detection
- Shape classification (ball, cylinder, cone)

### 3. Robot Control
The robot:
- Rotates to search for the target object
- Aligns itself with the object
- Moves forward while avoiding obstacles
- Stops when it reaches the target

Obstacle avoidance is handled using proximity sensors.

---

## Project Structure
