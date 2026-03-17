#Autonomous Drone Person Tracking
An end-to-end autonomous drone tracking system that integrates advanced computer vision, real-time drone telemetry, and closed-loop control. Developed in association with OEG DEFENCE & AEROSPACE Pvt. Ltd., this project enables a drone to identify, lock onto, and autonomously track a user-selected person in real-time.

🚀 Overview
This system bridges the gap between deep learning and UAV flight control. By leveraging the Ultralytics YOLOv8 model, the system detects humans within the drone's live video stream. Once a target is selected, a Proportional-Integral-Derivative (PID) controller calculates the necessary yaw adjustments to keep the target perfectly centered in the frame.

To ensure system stability and prevent hardware overload, GPS data retrieval is dynamically throttled to a configurable rate. This multithreaded architecture balances the need for fresh telemetry with the computational demands of real-time object detection.

✨ Key Features
Real-Time Subject Tracking: Utilizes YOLOv8 for high-speed, accurate human detection from the drone's camera feed.

Autonomous Centering (PID Control): Implements a custom PID controller that continuously calculates positional errors (bounding box center vs. frame center) and issues precise yaw commands via DroneKit to keep the target in view.

Smart Telemetry Management: Features a live GPS update rate manager that throttles data requests, optimizing the balance between positional awareness and CPU load.

Multithreaded Architecture: Separates the video processing, flight control, and telemetry logging into distinct threads to ensure smooth, non-blocking execution.

Auditory Feedback System: Integrates pyttsx3 for text-to-speech, providing the operator with live auditory status updates (e.g., "Target Locked," "Tracking Lost," "GPS Updated").

🛠️ Tech Stack & Skills
Language: Python

Computer Vision: OpenCV, Ultralytics YOLOv8

UAV Control: DroneKit-Python, MAVLink

Control Systems: PID Controller logic

Concurrency: Python threading module

Feedback: pyttsx3 (Text-to-Speech)

🧠 System Architecture
Vision Node: OpenCV captures the live video stream. YOLOv8 processes each frame to output bounding boxes for detected humans.

Tracking Logic: The user selects a specific bounding box. The system calculates the pixel offset between the target's center and the camera's center.

Control Node: The pixel offset is fed into the PID controller as the "error." The controller outputs a rotational velocity (yaw rate).

Command Execution: DroneKit translates the yaw rate into MAVLink commands sent to the drone's flight controller (e.g., Pixhawk/ArduPilot).

Telemetry Node: A background thread periodically fetches GPS and altitude data at a throttled rate to log the drone's tracking path without interrupting the vision loop.
