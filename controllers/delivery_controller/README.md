# Context-Nav-Bot: Delivery Controller

An autonomous delivery robot controller for Webots simulation, featuring AI-powered navigation and computer vision for traffic handling.

## Overview

This project implements a delivery robot that can:
- Navigate to destinations based on natural language commands (e.g., "I'm hungry" → restaurant)
- Detect and respond to traffic lights using YOLO object detection
- Identify crosswalks and wait for safe crossing conditions
- Avoid obstacles using lidar-based navigation
- Recover from stuck situations using a watchdog system

## Architecture

| File | Description |
|------|-------------|
| `delivery_controller.py` | Main controller with state machine logic |
| `llm_brain.py` | Gemini AI integration for destination parsing |
| `navigation.py` | Hardware abstraction layer for TIAGo robot |
| `vision_brain.py` | Computer vision (YOLO + OpenCV) |

## State Machine

The robot operates using the following states:

1. **CRUISING** - Normal navigation toward target
2. **SCANNING** - Rotating to find traffic lights at crosswalk
3. **WAITING** - Stopped, waiting for green light
4. **ALIGNING** - Rotating to face target direction
5. **COMMITTING** - Crossing intersection with collision avoidance
6. **PICKUP** - Simulated package pickup at destination
7. **RETREAT** - Recovery from blocked paths
8. **WATCHDOG** - Escape sequence when stuck

## Requirements

- Python 3.8+
- Webots R2023b or later
- Dependencies:
  ```
  google-genai
  ultralytics
  opencv-python
  numpy
  ```

## Setup

1. **API Key Configuration**
   
   Create `my_secrets.py` with your Gemini API key:
   ```python
   GEMINI_API_KEY = "your-api-key-here"
   ```

2. **YOLO Model**
   
   Ensure `yolo26n.pt` is in the controller directory.

3. **Webots World**
   
   Open the corresponding Webots world file and run the simulation.

## Usage

Run the simulation in Webots. A popup dialog will appear asking for your destination:
- "I'm hungry" → Routes to restaurant
- "Go to the hospital" → Routes to hospital
- "I need to work" → Routes to office

The robot will navigate autonomously, handling traffic lights and obstacles.

## Known Locations

| Zone | Coordinates | Description |
|------|-------------|-------------|
| residential | (85.2, -5.14) | Home base |
| park | (0.0, 0.0) | Recreation area |
| shopping_mall | (1.3, 55.8) | Shopping center |
| hospital | (-88, 4.98) | Medical facility |
| restaurant | (87.2, -59.8) | Food service |
| gas_station | (84.6, -72.4) | Fuel station |
| office | (-72.9, 68.9) | Workplace |
| museum | (82.3, 90.5) | Cultural venue |
| church | (-91.5, -80.6) | Religious site |
