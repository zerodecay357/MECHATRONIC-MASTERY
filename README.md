# MECHATRONIC-MASTERY

# Holonomic Robot Navigation - Task 1

## Overview
Autonomous holonomic robot navigates through 5 waypoints using **A* path planning** with smooth motion control and obstacle avoidance.

## Robot Specifications
- **Type:** 4-wheel omnidirectional platform
- **Sensors:** 16 proximity sensors (360° coverage)
- **Max Speed:** 2.0 m/s
- **Robot Radius:** 0.175 m

## Control Algorithm

### A* Path Planning
- Grid resolution: 0.15 m
- Obstacle inflation: 0.25 m safety margin
- 8-connected grid with Euclidean heuristic
- Automatic path smoothing enabled

### Motion Control
- Smooth velocity interpolation (factor: 0.08)
- Adaptive deceleration near waypoints
- Multi-layer obstacle avoidance (global + local)
- Automatic boundary detection and protection

### Stuck Recovery
- Detects if movement < 0.15m in 3 seconds
- Executes perpendicular escape + replanning

## Waypoints
| # | X | Y |
|---|-------|---------|
| 1 | 1.575 | 0.9 |
| 2 | -1.425 | 1.525 |
| 3 | -0.125 | -0.075 |
| 4 | 0.225 | -1.65 |
| 5 | -1.85 | -0.7 |

## Key Features
- **Optimal pathfinding:** A* guarantees shortest grid path
- **Smooth motion:** Eliminates jitter with velocity interpolation
- **Robust avoidance:** 3-layer system (global/local/boundary)
- **Auto-recovery:** Detects and escapes stuck situations

## Omni-Wheel Kinematics
v₁ = (vₓ - vᵧ - r·ω) / R
v₂ = (vₓ + vᵧ + r·ω) / R
v₃ = (vₓ - vᵧ + r·ω) / R
v₄ = (vₓ + vᵧ - r·ω) / R
*where r=0.175m, R=0.04m*

## Main Functions
- `aStarPathfinding()` - Optimal path generation
- `smoothPath()` - 4-point weighted averaging
- `smoothVelocity()` - Gradual acceleration/deceleration
- `setOmniWheelVelocities()` - Inverse kinematics
- `checkBoundaries()` - Arena edge protection

# Manipulator Motion Control - Task 2

## Overview
Robotic manipulator sphere traces multiple predefined motion patterns with dynamic parameter control via Python API. Real-time validation ensures the tip follows the sphere accurately.

## System Components
- **Manipulator Sphere (Dummy):** Target object tracing motion patterns
- **Tip:** End-effector that must follow the sphere
- **Control Interface:** Python API for real-time parameter adjustment

## Motion Modes

### 1. Sine Wave
- Linear X-axis motion (0 to r and back)
- Sinusoidal Y-axis oscillation
- **Parameters:** amplitude, frequency, r

### 2. Square Pattern
- Traces 4-sided square path
- 4 equal segments (25% each per cycle)
- **Parameter:** r (side length)

### 3. Circle Pattern
- Circular trajectory in X-Y plane
- Completes full circle per cycle
- **Parameter:** r (radius)

### 4. Reach Point
- Linear interpolation from start to end position
- Smooth trajectory over 30 seconds
- **Parameter:** endPos (target coordinates)

## Key Parameters

| Parameter | Default | Range | Unit |
|-----------|---------|-------|------|
| r | 0.1 | 0.05-0.5 | m |
| amplitude | 0.1 | 0.01-0.3 | m |
| frequency | 2 | 1-5 | Hz |
| duration | 30 | - | seconds |

## Control Interface (Python API)
-- Set via Python:
sim.setFloatProperty(scene_handle, "signal.r", 0.15)
sim.setFloatProperty(scene_handle, "signal.amplitude", 0.12)
sim.setFloatProperty(scene_handle, "signal.frequency", 3)
sim.setStringProperty(scene_handle, "signal.motionMode", "circle")

## Reachability Validation
- Checks distance between sphere and tip every step
- **Threshold:** 0.05 m tolerance
- **Action:** Pauses simulation if exceeded
- **Message:** "Can't be reached"

## Motion Cycle
- **Duration:** 30 seconds per cycle
- **Auto-repeat:** Restarts when ratio ≥ 1.0
- **Progress:** Ratio = current_time / 30

## Code Structure
- `sysCall_init()` - Initialize handles, positions, parameters
- `sysCall_actuation()` - Main control loop, parameter updates, validation
- `sine()` - Sine wave trajectory
- `square()` - Square path generator
- `circle()` - Circular motion
- `reachPoint()` - Linear interpolation to target

## Mathematical Formulations

**Sine Wave:**
x = x₀ + r × (2t/T)
y = y₀ + A × sin(2πf × t)

**Circle:**
x = x₀ - r(1 - cos(2πt))
y = y₀ + r × sin(2πt)

**Square (each segment = T/4):**
- Segment 1: Move +r in X
- Segment 2: Move +r in Y
- Segment 3: Move -r in X
- Segment 4: Move -r in Y

