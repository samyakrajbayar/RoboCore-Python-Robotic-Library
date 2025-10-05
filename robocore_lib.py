"""
RoboCore - Advanced Python Robotics Library
A comprehensive library for building and controlling robots with sophisticated algorithms.
"""

import math
import time
from typing import List, Tuple, Callable, Optional, Dict
from dataclasses import dataclass
from enum import Enum
import threading
import queue


# ==================== CORE COMPONENTS ====================

class MotorType(Enum):
    """Motor types supported by the library"""
    DC = "dc"
    SERVO = "servo"
    STEPPER = "stepper"


@dataclass
class Vector3D:
    """3D vector representation for position and orientation"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def magnitude(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def normalize(self) -> 'Vector3D':
        mag = self.magnitude()
        if mag == 0:
            return Vector3D(0, 0, 0)
        return Vector3D(self.x/mag, self.y/mag, self.z/mag)
    
    def __add__(self, other: 'Vector3D') -> 'Vector3D':
        return Vector3D(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other: 'Vector3D') -> 'Vector3D':
        return Vector3D(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, scalar: float) -> 'Vector3D':
        return Vector3D(self.x * scalar, self.y * scalar, self.z * scalar)


class Motor:
    """Base motor class with PWM control and acceleration curves"""
    
    def __init__(self, pin: int, motor_type: MotorType, max_speed: float = 100.0):
        self.pin = pin
        self.motor_type = motor_type
        self.max_speed = max_speed
        self.current_speed = 0.0
        self.target_speed = 0.0
        self.acceleration = 10.0  # units per second
        
    def set_speed(self, speed: float, smooth: bool = True):
        """Set motor speed with optional smooth acceleration"""
        speed = max(-self.max_speed, min(self.max_speed, speed))
        self.target_speed = speed
        
        if not smooth:
            self.current_speed = speed
            self._apply_speed()
    
    def update(self, dt: float):
        """Update motor state for smooth acceleration"""
        if abs(self.current_speed - self.target_speed) > 0.1:
            diff = self.target_speed - self.current_speed
            step = self.acceleration * dt
            
            if abs(diff) < step:
                self.current_speed = self.target_speed
            else:
                self.current_speed += step if diff > 0 else -step
            
            self._apply_speed()
    
    def _apply_speed(self):
        """Apply current speed to hardware (override in implementation)"""
        pass
    
    def stop(self):
        """Emergency stop"""
        self.current_speed = 0.0
        self.target_speed = 0.0
        self._apply_speed()


class Sensor:
    """Base sensor class with filtering and calibration"""
    
    def __init__(self, pin: int, sensor_type: str):
        self.pin = pin
        self.sensor_type = sensor_type
        self.calibration_offset = 0.0
        self.filter_alpha = 0.3  # Low-pass filter coefficient
        self.filtered_value = 0.0
        
    def read_raw(self) -> float:
        """Read raw sensor value (override in implementation)"""
        return 0.0
    
    def read(self) -> float:
        """Read filtered and calibrated value"""
        raw = self.read_raw()
        self.filtered_value = (self.filter_alpha * raw + 
                              (1 - self.filter_alpha) * self.filtered_value)
        return self.filtered_value + self.calibration_offset
    
    def calibrate(self, samples: int = 100):
        """Auto-calibrate sensor by averaging samples"""
        total = sum(self.read_raw() for _ in range(samples))
        self.calibration_offset = -(total / samples)


# ==================== KINEMATICS ====================

class RobotArm:
    """Robotic arm with forward and inverse kinematics"""
    
    def __init__(self, link_lengths: List[float]):
        self.link_lengths = link_lengths
        self.joint_angles = [0.0] * len(link_lengths)
        self.num_joints = len(link_lengths)
    
    def forward_kinematics(self) -> Vector3D:
        """Calculate end-effector position from joint angles"""
        x, y, z = 0.0, 0.0, 0.0
        angle_sum = 0.0
        
        for i, (length, angle) in enumerate(zip(self.link_lengths, self.joint_angles)):
            angle_sum += angle
            x += length * math.cos(angle_sum)
            y += length * math.sin(angle_sum)
        
        return Vector3D(x, y, z)
    
    def inverse_kinematics_2d(self, target: Vector3D, elbow_up: bool = True) -> List[float]:
        """2-link inverse kinematics using geometric solution"""
        if self.num_joints != 2:
            raise ValueError("2D IK only works with 2-joint arms")
        
        L1, L2 = self.link_lengths[0], self.link_lengths[1]
        x, y = target.x, target.y
        
        # Distance to target
        d = math.sqrt(x*x + y*y)
        
        # Check if target is reachable
        if d > L1 + L2 or d < abs(L1 - L2):
            raise ValueError("Target out of reach")
        
        # Law of cosines
        cos_angle2 = (d*d - L1*L1 - L2*L2) / (2 * L1 * L2)
        angle2 = math.acos(cos_angle2) if elbow_up else -math.acos(cos_angle2)
        
        # Calculate angle1
        k1 = L1 + L2 * math.cos(angle2)
        k2 = L2 * math.sin(angle2)
        angle1 = math.atan2(y, x) - math.atan2(k2, k1)
        
        return [angle1, angle2]
    
    def set_joint_angles(self, angles: List[float]):
        """Set joint angles directly"""
        self.joint_angles = angles[:self.num_joints]
    
    def move_to_position(self, target: Vector3D, elbow_up: bool = True):
        """Move arm to target position using IK"""
        angles = self.inverse_kinematics_2d(target, elbow_up)
        self.set_joint_angles(angles)


# ==================== PATH PLANNING ====================

class PathPlanner:
    """Advanced path planning algorithms"""
    
    @staticmethod
    def interpolate_linear(start: Vector3D, end: Vector3D, steps: int) -> List[Vector3D]:
        """Linear interpolation between two points"""
        path = []
        for i in range(steps + 1):
            t = i / steps
            point = Vector3D(
                start.x + (end.x - start.x) * t,
                start.y + (end.y - start.y) * t,
                start.z + (end.z - start.z) * t
            )
            path.append(point)
        return path
    
    @staticmethod
    def bezier_curve(control_points: List[Vector3D], steps: int) -> List[Vector3D]:
        """Generate smooth Bezier curve path"""
        path = []
        n = len(control_points) - 1
        
        for i in range(steps + 1):
            t = i / steps
            point = Vector3D(0, 0, 0)
            
            for j, cp in enumerate(control_points):
                # Bernstein polynomial
                coeff = (math.comb(n, j) * 
                        math.pow(1 - t, n - j) * 
                        math.pow(t, j))
                point = point + cp * coeff
            
            path.append(point)
        return path
    
    @staticmethod
    def circular_arc(center: Vector3D, radius: float, start_angle: float, 
                     end_angle: float, steps: int) -> List[Vector3D]:
        """Generate circular arc path"""
        path = []
        angle_range = end_angle - start_angle
        
        for i in range(steps + 1):
            angle = start_angle + (angle_range * i / steps)
            point = Vector3D(
                center.x + radius * math.cos(angle),
                center.y + radius * math.sin(angle),
                center.z
            )
            path.append(point)
        return path


# ==================== PID CONTROLLER ====================

class PIDController:
    """PID controller with anti-windup and derivative filtering"""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_limits: Tuple[float, float] = (-100, 100)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()
        
    def compute(self, setpoint: float, measured: float) -> float:
        """Compute PID control output"""
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            return 0.0
        
        error = setpoint - measured
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply limits
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        
        # Anti-windup: prevent integral from growing if output is saturated
        if output == self.output_limits[0] or output == self.output_limits[1]:
            self.integral -= error * dt
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()


# ==================== ODOMETRY & LOCALIZATION ====================

class DifferentialDriveOdometry:
    """Odometry for differential drive robots"""
    
    def __init__(self, wheel_base: float, wheel_radius: float):
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.position = Vector3D(0, 0, 0)
        self.heading = 0.0  # radians
        
    def update(self, left_encoder: float, right_encoder: float):
        """Update position based on encoder readings (in radians)"""
        # Calculate distances
        left_dist = left_encoder * self.wheel_radius
        right_dist = right_encoder * self.wheel_radius
        
        # Calculate robot movement
        center_dist = (left_dist + right_dist) / 2.0
        delta_heading = (right_dist - left_dist) / self.wheel_base
        
        # Update heading
        self.heading += delta_heading
        
        # Update position
        self.position.x += center_dist * math.cos(self.heading)
        self.position.y += center_dist * math.sin(self.heading)
        
    def reset(self):
        """Reset odometry to origin"""
        self.position = Vector3D(0, 0, 0)
        self.heading = 0.0
    
    def get_pose(self) -> Tuple[Vector3D, float]:
        """Get current position and heading"""
        return self.position, self.heading


# ==================== STATE MACHINE ====================

class State:
    """Base state for robot state machines"""
    
    def on_enter(self):
        """Called when entering the state"""
        pass
    
    def update(self, dt: float):
        """Called every update cycle"""
        pass
    
    def on_exit(self):
        """Called when exiting the state"""
        pass


class StateMachine:
    """Finite state machine for robot behavior control"""
    
    def __init__(self):
        self.states: Dict[str, State] = {}
        self.current_state: Optional[State] = None
        self.current_state_name: Optional[str] = None
        
    def add_state(self, name: str, state: State):
        """Add a state to the machine"""
        self.states[name] = state
    
    def transition_to(self, state_name: str):
        """Transition to a new state"""
        if state_name not in self.states:
            raise ValueError(f"State '{state_name}' not found")
        
        if self.current_state:
            self.current_state.on_exit()
        
        self.current_state = self.states[state_name]
        self.current_state_name = state_name
        self.current_state.on_enter()
    
    def update(self, dt: float):
        """Update current state"""
        if self.current_state:
            self.current_state.update(dt)


# ==================== TRAJECTORY GENERATION ====================

class TrajectoryGenerator:
    """Generate smooth trajectories with velocity and acceleration profiles"""
    
    @staticmethod
    def trapezoidal_profile(distance: float, max_vel: float, max_accel: float) -> Dict:
        """Generate trapezoidal velocity profile"""
        # Calculate acceleration time
        t_accel = max_vel / max_accel
        d_accel = 0.5 * max_accel * t_accel**2
        
        # Check if we reach max velocity
        if 2 * d_accel > distance:
            # Triangular profile
            t_accel = math.sqrt(distance / max_accel)
            actual_max_vel = max_accel * t_accel
            t_cruise = 0
            t_total = 2 * t_accel
        else:
            # Trapezoidal profile
            actual_max_vel = max_vel
            d_cruise = distance - 2 * d_accel
            t_cruise = d_cruise / max_vel
            t_total = 2 * t_accel + t_cruise
        
        return {
            'total_time': t_total,
            'accel_time': t_accel,
            'cruise_time': t_cruise,
            'max_velocity': actual_max_vel,
            'acceleration': max_accel
        }
    
    @staticmethod
    def get_position_at_time(profile: Dict, t: float) -> float:
        """Get position at time t for a trapezoidal profile"""
        t_accel = profile['accel_time']
        t_cruise = profile['cruise_time']
        max_vel = profile['max_velocity']
        accel = profile['acceleration']
        
        if t < 0:
            return 0.0
        elif t < t_accel:
            # Acceleration phase
            return 0.5 * accel * t**2
        elif t < t_accel + t_cruise:
            # Cruise phase
            t_in_cruise = t - t_accel
            d_accel = 0.5 * accel * t_accel**2
            return d_accel + max_vel * t_in_cruise
        elif t < profile['total_time']:
            # Deceleration phase
            t_in_decel = t - t_accel - t_cruise
            d_accel = 0.5 * accel * t_accel**2
            d_cruise = max_vel * t_cruise
            return d_accel + d_cruise + max_vel * t_in_decel - 0.5 * accel * t_in_decel**2
        else:
            # Past end of motion
            d_accel = 0.5 * accel * t_accel**2
            d_cruise = max_vel * t_cruise
            return 2 * d_accel + d_cruise


# ==================== MULTI-THREADING ROBOT CONTROLLER ====================

class RobotController:
    """Main robot controller with multi-threaded execution"""
    
    def __init__(self, update_rate: float = 50.0):
        self.update_rate = update_rate
        self.running = False
        self.thread = None
        self.command_queue = queue.Queue()
        self.motors: List[Motor] = []
        self.sensors: List[Sensor] = []
        
    def add_motor(self, motor: Motor):
        """Add a motor to the controller"""
        self.motors.append(motor)
    
    def add_sensor(self, sensor: Sensor):
        """Add a sensor to the controller"""
        self.sensors.append(sensor)
    
    def start(self):
        """Start the control loop in a separate thread"""
        self.running = True
        self.thread = threading.Thread(target=self._control_loop)
        self.thread.start()
    
    def stop(self):
        """Stop the control loop"""
        self.running = False
        if self.thread:
            self.thread.join()
        for motor in self.motors:
            motor.stop()
    
    def _control_loop(self):
        """Main control loop running in separate thread"""
        dt = 1.0 / self.update_rate
        
        while self.running:
            start_time = time.time()
            
            # Process commands from queue
            while not self.command_queue.empty():
                try:
                    cmd = self.command_queue.get_nowait()
                    cmd()
                except queue.Empty:
                    break
            
            # Update all motors
            for motor in self.motors:
                motor.update(dt)
            
            # Sleep to maintain update rate
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def queue_command(self, command: Callable):
        """Queue a command to be executed in the control loop"""
        self.command_queue.put(command)


# ==================== EXAMPLE USAGE ====================

def example_usage():
    """Example demonstrating library features"""
    
    # Create a 2-link robot arm
    arm = RobotArm([10.0, 8.0])
    
    # Calculate forward kinematics
    arm.set_joint_angles([math.pi/4, math.pi/4])
    end_pos = arm.forward_kinematics()
    print(f"End effector position: ({end_pos.x:.2f}, {end_pos.y:.2f})")
    
    # Inverse kinematics
    target = Vector3D(12.0, 5.0, 0.0)
    try:
        arm.move_to_position(target)
        print(f"Joint angles: {[math.degrees(a) for a in arm.joint_angles]}")
    except ValueError as e:
        print(f"IK error: {e}")
    
    # Path planning
    start = Vector3D(0, 0, 0)
    end = Vector3D(10, 10, 0)
    path = PathPlanner.interpolate_linear(start, end, 20)
    print(f"Generated path with {len(path)} points")
    
    # PID controller
    pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
    output = pid.compute(setpoint=100, measured=75)
    print(f"PID output: {output:.2f}")
    
    # Trajectory generation
    profile = TrajectoryGenerator.trapezoidal_profile(
        distance=100, max_vel=50, max_accel=20
    )
    print(f"Trajectory time: {profile['total_time']:.2f}s")
    
    # Odometry
    odom = DifferentialDriveOdometry(wheel_base=0.3, wheel_radius=0.05)
    odom.update(left_encoder=1.0, right_encoder=1.2)
    pos, heading = odom.get_pose()
    print(f"Robot position: ({pos.x:.2f}, {pos.y:.2f}), heading: {math.degrees(heading):.1f}°")


if __name__ == "__main__":
    example_usage()
