# 🤖 RoboCore - Advanced Python Robotics Library

A comprehensive, production-ready Python library for building and controlling robots with sophisticated algorithms for kinematics, path planning, control systems, and localization.

[![Python Version](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-active-success.svg)]()

## 📋 Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Core Components](#core-components)
- [Advanced Features](#advanced-features)
- [Examples](#examples)
- [API Reference](#api-reference)
- [Contributing](#contributing)
- [License](#license)

## ✨ Features

### Core Capabilities

- **Motor Control**: Advanced motor control with smooth acceleration curves, PWM support, and multiple motor types (DC, Servo, Stepper)
- **Sensor Management**: Unified sensor interface with filtering, calibration, and noise reduction
- **Kinematics**: Forward and inverse kinematics for robotic arms with geometric solutions
- **Path Planning**: Multiple path planning algorithms including linear interpolation, Bezier curves, and circular arcs
- **PID Control**: Industrial-grade PID controller with anti-windup and derivative filtering
- **Odometry**: Differential drive odometry for accurate position tracking
- **State Machines**: Flexible finite state machine for complex robot behaviors
- **Trajectory Generation**: Smooth motion profiles with trapezoidal velocity curves
- **Multi-threading**: Thread-safe robot controller with command queuing

### Advanced Features

- 3D vector mathematics with normalization and operations
- Real-time control loop with configurable update rates
- Low-pass filtering for sensor noise reduction
- Auto-calibration routines for sensors
- Reachability checking for inverse kinematics
- Bezier curve path smoothing for elegant robot motion
- Anti-windup protection in PID controllers
- Thread-safe command queuing system

## 🚀 Installation

### Requirements

- Python 3.8 or higher
- No external dependencies required (pure Python implementation)

### Install from source

```bash
git clone https://github.com/yourusername/robocore.git
cd robocore
pip install -e .
```

### Quick install

```bash
pip install robocore
```

## 🎯 Quick Start

### Basic Robot Arm Control

```python
from robocore import RobotArm, Vector3D
import math

# Create a 2-link robot arm (link lengths: 10cm and 8cm)
arm = RobotArm([10.0, 8.0])

# Set joint angles (in radians)
arm.set_joint_angles([math.pi/4, math.pi/4])

# Calculate end effector position
position = arm.forward_kinematics()
print(f"End effector at: ({position.x:.2f}, {position.y:.2f})")

# Move to target position using inverse kinematics
target = Vector3D(12.0, 5.0, 0.0)
arm.move_to_position(target)
print(f"Joint angles: {arm.joint_angles}")
```

### Motor Control with Smooth Acceleration

```python
from robocore import Motor, MotorType

# Create a DC motor
motor = Motor(pin=3, motor_type=MotorType.DC, max_speed=100.0)

# Set speed with smooth acceleration
motor.set_speed(75.0, smooth=True)

# Update motor in control loop
dt = 0.02  # 50Hz update rate
motor.update(dt)
```

### PID Control System

```python
from robocore import PIDController

# Create PID controller (kp, ki, kd)
pid = PIDController(kp=1.0, ki=0.1, kd=0.05, output_limits=(-100, 100))

# In control loop
setpoint = 100.0
current_position = sensor.read()
control_output = pid.compute(setpoint, current_position)
motor.set_speed(control_output)
```

### Path Planning

```python
from robocore import PathPlanner, Vector3D

# Linear path
start = Vector3D(0, 0, 0)
end = Vector3D(100, 50, 0)
path = PathPlanner.interpolate_linear(start, end, steps=50)

# Smooth Bezier curve
control_points = [
    Vector3D(0, 0, 0),
    Vector3D(25, 50, 0),
    Vector3D(75, 50, 0),
    Vector3D(100, 0, 0)
]
smooth_path = PathPlanner.bezier_curve(control_points, steps=100)

# Circular arc
center = Vector3D(50, 50, 0)
arc = PathPlanner.circular_arc(center, radius=30, 
                                start_angle=0, end_angle=math.pi, 
                                steps=50)
```

## 🔧 Core Components

### Vector3D

3D vector class for position, velocity, and direction representation.

```python
v1 = Vector3D(3, 4, 0)
v2 = Vector3D(1, 1, 0)

# Operations
v3 = v1 + v2
magnitude = v1.magnitude()
normalized = v1.normalize()
scaled = v1 * 2.5
```

### Motor

Base motor class with acceleration control.

```python
motor = Motor(pin=5, motor_type=MotorType.SERVO, max_speed=180)
motor.acceleration = 50.0  # Set acceleration rate
motor.set_speed(90.0, smooth=True)
motor.update(0.02)  # Update in control loop
motor.stop()  # Emergency stop
```

### Sensor

Unified sensor interface with filtering.

```python
sensor = Sensor(pin=A0, sensor_type="ultrasonic")
sensor.filter_alpha = 0.3  # Adjust filtering strength

# Calibration
sensor.calibrate(samples=100)

# Reading
filtered_value = sensor.read()
raw_value = sensor.read_raw()
```

## 🎓 Advanced Features

### Robot Arm Kinematics

Forward kinematics calculates the end effector position from joint angles:

```python
arm = RobotArm([15.0, 10.0, 8.0])  # 3-link arm
arm.set_joint_angles([0.5, 0.3, 0.2])
end_position = arm.forward_kinematics()
```

Inverse kinematics solves for joint angles to reach a target position:

```python
target = Vector3D(20.0, 10.0, 0.0)
try:
    arm.move_to_position(target, elbow_up=True)
    print("Target reached!")
except ValueError as e:
    print(f"Target unreachable: {e}")
```

### Differential Drive Odometry

Track robot position using wheel encoders:

```python
odom = DifferentialDriveOdometry(wheel_base=0.3, wheel_radius=0.05)

# In control loop, update with encoder readings
odom.update(left_encoder=1.57, right_encoder=1.60)

# Get current pose
position, heading = odom.get_pose()
print(f"Position: ({position.x:.2f}, {position.y:.2f})")
print(f"Heading: {math.degrees(heading):.1f}°")
```

### Trajectory Generation

Generate smooth motion profiles:

```python
from robocore import TrajectoryGenerator

# Create trapezoidal velocity profile
profile = TrajectoryGenerator.trapezoidal_profile(
    distance=100.0,
    max_vel=50.0,
    max_accel=20.0
)

print(f"Motion time: {profile['total_time']:.2f}s")
print(f"Max velocity: {profile['max_velocity']:.2f}")

# Get position at specific time
t = 1.5
position = TrajectoryGenerator.get_position_at_time(profile, t)
```

### State Machine

Create complex robot behaviors:

```python
from robocore import StateMachine, State

class IdleState(State):
    def on_enter(self):
        print("Robot idle")
    
    def update(self, dt):
        # Check for transition conditions
        pass

class MovingState(State):
    def on_enter(self):
        print("Starting movement")
    
    def update(self, dt):
        # Move robot
        pass
    
    def on_exit(self):
        print("Stopping movement")

# Create and use state machine
sm = StateMachine()
sm.add_state("idle", IdleState())
sm.add_state("moving", MovingState())
sm.transition_to("idle")

# In control loop
sm.update(dt=0.02)
```
