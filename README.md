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

## Video Demonstration
See `video_link.txt` for complete demonstration showing initialization, path planning, waypoint navigation, and completion statistics.

