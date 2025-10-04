# FRC Advanced Path Planning System

A real-time, on-the-fly path planning system for FRC robots using A* pathfinding with dynamic obstacle avoidance, designed to run on Jetson Orin Nano Super and integrate with FRC-AI-Vision.

## Features

- **A* Pathfinding with Visibility Graphs**: Efficient pathfinding around static and dynamic obstacles
- **Real-time Replanning**: Continuously updates paths as obstacles move or appear
- **Vision Integration**: Uses object detection from FRC-AI-Vision for dynamic obstacle detection
- **Velocity Profiling**: Generates time-parameterized trajectories respecting robot dynamics
- **Path Optimization**: Smooths paths using Bezier curves and optimizes for minimal curvature
- **NetworkTables Communication**: Seamless integration with roboRIO via NetworkTables
- **Autonomous & Teleoperated**: Supports both auto period and driver-assisted navigation

## Architecture


## Quick Start

### Prerequisites

- Java 17 (Gradle toolchain will download if not present)
- WPILib 2024+
- Jetson Orin Nano Super
- FRC-AI-Vision repository

### Installation

1. Clone this repository:
```bash
git clone https://github.com/Antigro09/1086-On-The-Fly-A-Star.git
cd 1086-On-The-Fly-A-Star
```

2. Build the project (compiles sources and creates `build/libs/on-the-fly-a-star-0.1.0.jar`):
```bash
./gradlew build
```

3. (Optional) Run the autonomous example locally:
```bash
./gradlew run
```
or, if you prefer to launch directly from the packaged jar (after running `./gradlew build`):
```bash
java -cp build/libs/on-the-fly-a-star-0.1.0.jar pathplanning.examples.AutoPathPlanning
```
    To run another entry point, override the main class:
```bash
./gradlew run -PmainClass=pathplanning.examples.TeleopPathPlanning
```
or
```bash
java -cp build/libs/on-the-fly-a-star-0.1.0.jar pathplanning.examples.TeleopPathPlanning
```
Configure field obstacles in config/field_config.json

Adjust planning parameters in config/planner_params.yaml

### Running on Jetson
For autonomous planning:

```bash
./gradlew run
```
For teleoperated planning:

```bash
./gradlew run -PmainClass=pathplanning.examples.TeleopPathPlanning
```

## Integration with Robot Code
On roboRIO, use FollowDynamicPath command:

```Java
// Set target pose
networkTables.getTable("pathplanner")
    .getEntry("target_pose")
    .setDoubleArray(new double[]{x, y, rotation});

// Follow dynamically planned path
new FollowDynamicPath(drivetrain).schedule();
```