# 6-DOF Robotic Arm — ROS 1 Noetic (Industrial Hardening Baseline)
### Industrial Reference Architecture: MoveIt! Motion Planning, Analytical IKFast, Gazebo Physics, and Active Safety Interlocks

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic%20(Ubuntu%2020.04)-brightgreen.svg?logo=ros&logoColor=white)](https://wiki.ros.org/noetic)
[![MoveIt 1](https://img.shields.io/badge/Motion%20Planning-MoveIt!%201.1-1B4F72.svg)](https://moveit.ros.org/)
[![Gazebo 11](https://img.shields.io/badge/Simulation-Gazebo%2011-orange.svg?logo=gazebo&logoColor=white)](https://gazebosim.org/)
[![Kinematics: IKFast](https://img.shields.io/badge/Kinematics-IKFast%20Analytical-purple.svg)](https://github.com/sam-black007/6-dof-robotic-arm-ros1-industrial)
[![CI Build](https://github.com/sam-black007/6-dof-robotic-arm-ros1-industrial/actions/workflows/ros1-ci.yml/badge.svg)](https://github.com/sam-black007/6-dof-robotic-arm-ros1-industrial/actions/workflows/ros1-ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](https://github.com/sam-black007/6-dof-robotic-arm-ros1-industrial/pulls)

A production-hardened industrial reference architecture for the **OWR 6-DOF articulated robotic manipulator** equipped with a **Robotiq 2F-140 parallel-jaw adaptive gripper**. 

Designed to transition academic ROS manipulators into predictable, industrially hardened workcells, this repository integrates a high-speed closed-form analytical inverse kinematics solver (**IKFast**), multi-pipeline motion planning (**OMPL, Pilz Industrial Planner, CHOMP, STOMP**), realistic Gazebo physics with high-frequency `ros_control` PID tuning, real-time 3D perception with **OctoMap**, and a dedicated multi-tiered **safety watchdog layer** with latched emergency-stop (`/emergency_stop`) mechanisms.

> [!NOTE]
> **Industrial Baseline Notice:** While ROS 1 Noetic is officially End-of-Life (EOL), this codebase serves as a validated, hardened engineering reference for maintaining existing production workcells, academic research rigs, and as a formal architectural blueprint for migration to ROS 2 (MoveIt 2 / `ros2_control`).

---

<p align="center">
  <img src="images/architecture.png" alt="System Architecture Overview" width="850">
</p>

---

## 📑 Table of Contents

- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Kinematic Transform Pipeline & Frame Tree](#-kinematic-transform-pipeline--frame-tree)
- [Robot Kinematic Specification](#-robot-kinematic-specification)
- [Software Stack & Package Ecosystem](#-software-stack--package-ecosystem)
- [Motion Planning & Inverse Kinematics Pipeline](#-motion-planning--inverse-kinematics-pipeline)
- [Closed-Loop Control & ROS Hardware Interface](#-closed-loop-control--ros-hardware-interface)
- [Real-Time 3D Perception & OctoMap Pipeline](#-real-time-3d-perception--octomap-pipeline)
- [Autonomous Pick-and-Place State Machine](#-autonomous-pick-and-place-state-machine)
- [Multi-Tiered Safety & Emergency Stop Sequence](#-multi-tiered-safety--emergency-stop-sequence)
- [Physical Hardware Implementation](#-physical-hardware-implementation)
- [Quickstart & Launch Recipes](#-quickstart--launch-recipes)
- [End-to-End CI/CD & Verification Workflow](#-end-to-end-cicd--verification-workflow)
- [Troubleshooting & Diagnostics](#-troubleshooting--diagnostics)
- [Engineering Documentation](#-engineering-documentation)
- [License & Authorship](#-license--authorship)

---

## ✨ Key Features

- **6-DOF Kinematic Chain with Prismatic Gripper:** High-precision URDF/Xacro modeling featuring collision geometry, realistic joint inertias, and a Robotiq 2F-140 gripper with continuous trajectory action control.
- **Analytical IKFast Solver Plugin:** Sub-millisecond closed-form inverse kinematics computation ($< 5\text{ ms}$ timeout, $0.005\text{ rad}$ search resolution), eliminating local minima and singularities inherent to numerical solvers like KDL.
- **Multi-Pipeline Motion Planning:** Seamless runtime switching between **Pilz Industrial Motion Planner** (deterministic `PTP`, `LIN`, `CIRC` trajectory segments), **OMPL** (RRTConnect, RRT\*, PRM\*), **CHOMP**, and **STOMP**.
- **Real-Time 3D Perception & Volumetric OctoMap:** Simulated RGB-D point cloud processing pipeline with voxel filtering (1 cm leaf) generating live 3D occupancy voxel grids (5 cm resolution) for dynamic collision avoidance.
- **Strict `ros_control` PID Tuning:** Dual `JointTrajectoryController` instances for arm and end-effector operating at 50 Hz with tuned feedforward and feedback gains under `PositionJointInterface`.
- **Active Safety Watchdog & Latched E-Stop:** Independent Python runtime daemon (`safety_node.py`) continuously validating joint positions against URDF physical bounds ($0.05\text{ rad}$ safety envelope) with a bi-directional latched emergency-stop broker.
- **Automated CI/CD:** GitHub Actions workflow compiling the complete workspace and validating dependency resolution on Ubuntu 20.04 LTS against ROS Noetic Desktop Full.

---

## 🏗 System Architecture

The workcell architecture decouples motion generation, execution monitoring, perception, and hardware abstraction into modular, verified layers:

```mermaid
flowchart TD
    subgraph App["Application & Orchestration Layer"]
        TASK["PickNPlace / ArmMotion Nodes<br/>(C++ MoveGroupInterface)"]
        SAFETY["Safety Watchdog Daemon<br/>(safety_node.py @ 10 Hz)"]
    end

    subgraph MoveItCore["MoveIt! 1.1 Motion Planning Framework"]
        MG["move_group Node<br/>Planning Scene Monitor"]
        IK["IKFast 6-DOF Plugin<br/>(Closed-Form Analytical IK)"]
        PLANNERS["Planning Pipelines:<br/>• Pilz Industrial (LIN, PTP, CIRC)<br/>• OMPL (RRT*, PRM*)<br/>• CHOMP / STOMP"]
        MG --- IK
        MG --- PLANNERS
    end

    subgraph Perception["3D Perception Pipeline"]
        CAMERA["Simulated RGB-D Sensor<br/>(/camera/point_cloud @ 30 Hz)"]
        VOXEL["PCL Voxel Grid Filter<br/>(Leaf Size: 1 cm @ 10 Hz)"]
        OCTOMAP["OctoMap Server<br/>(/world_octomap @ 5 cm voxels)"]
        CAMERA --> VOXEL --> OCTOMAP --> MG
    end

    subgraph Control["ros_control Execution Layer"]
        TRAJ_EXEC["Trajectory Execution Manager<br/>(FollowJointTrajectory Action)"]
        ARM_CTRL["arm_manipulator_controller<br/>(PositionJointInterface @ 50 Hz)"]
        GRIP_CTRL["gripper_trajectory_controller<br/>(PositionJointInterface @ 50 Hz)"]
        JSC["joint_state_controller<br/>(/joint_states @ 50 Hz)"]
        TRAJ_EXEC --> ARM_CTRL & GRIP_CTRL
    end

    subgraph PhysicsHardware["Physical & Simulation Actuation"]
        GAZEBO["Gazebo 11 Physics Simulator<br/>(gazebo_ros_control plugin)"]
        HARDWARE["Real Hardware Interface<br/>(Arduino Mega 2560 + RAMPS 1.4)"]
        ARM_CTRL & GRIP_CTRL --> GAZEBO
        ARM_CTRL & GRIP_CTRL -.-> HARDWARE
    end

    TASK -->|Plan & Execute Goals| MG
    MG --> TRAJ_EXEC
    GAZEBO --> JSC
    JSC -->|Live Joint Angles| SAFETY
    SAFETY -->|Violation Detected / Stop Signal| ESTOP[("/emergency_stop Latch")]
    ESTOP -.->|Halt Trajectory| TRAJ_EXEC
```

<p align="center">
  <img src="images/hardening_summary.png" alt="Hardening Summary Matrix" width="760">
</p>

---

## 📐 Kinematic Transform Pipeline & Frame Tree

The robot kinematics are modeled via a strictly ordered homogeneous transform tree ($4 \times 4$ transform matrices) from the global `world` frame to the tool center point (`finger_joint`):

```mermaid
flowchart LR
    W["world"] -->|Fixed: 0, 0, 0.75m| B["base_link"]
    B -->|BJ (Rot Z)<br/>0, 0, 0.1181m<br/>[-120°, +120°]| L1["BJ_Link"]
    L1 -->|SJ (Rot Z)<br/>0, 0.1157, 0.0775m<br/>[-90°, +90°]| L2["SJ_Link"]
    L2 -->|EJ (Rot Z)<br/>0, 0, 0.35575m<br/>[-225°, +60°]| L3["EJ_Link"]
    L3 -->|W1J (Rot Z)<br/>0.0695, -0.1157, 0m<br/>[-90°, +90°]| L4["W1J_Link"]
    L4 -->|W2J (Rot Z)<br/>0.28625, 0, 0m<br/>[-60°, +150°]| L5["W2J_Link"]
    L5 -->|W3J (Rot Z)<br/>0.0635, 0, 0.1200m<br/>[-180°, +180°]| L6["W3J_Link"]
    L6 -->|Fixed Flange<br/>0.0675, 0, 0m| EEF["EEF_Link"]
    EEF -->|finger_joint (Prismatic Z)<br/>0.000 to 0.040m| GRIP["Robotiq 2F-140 Jaw"]
```

---

## 🦾 Robot Kinematic Specification

The OWR manipulator is a 6-DOF articulated serial arm with 6 revolute joints (all rotating about their respective local Z-axes) coupled with a Robotiq 2F-140 prismatic parallel gripper. Total reach extends to **~560 mm** from base origin to the end-effector mounting flange.

<p align="center">
  <img src="images/robot_model_preview.png" alt="Robot Model Preview" width="620">
</p>

### Joint Bounds & Physical Parameters

| Joint | Joint Name | Motion Type | Axis | Angular Limits (deg) | Angular Limits (rad) | Velocity Limit | Effort Limit | Link Mass |
|:---:|:---|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **$J_1$** | `BJ` (Base) | Revolute | Z | $-120^\circ \dots +120^\circ$ | $[-2.0944, +2.0944]$ | $5.0\text{ rad/s}$ | $200\text{ N}\cdot\text{m}$ | $2.004\text{ kg}$ |
| **$J_2$** | `SJ` (Shoulder) | Revolute | Z | $-90^\circ \dots +90^\circ$ | $[-1.5708, +1.5708]$ | $5.0\text{ rad/s}$ | $200\text{ N}\cdot\text{m}$ | $1.976\text{ kg}$ |
| **$J_3$** | `EJ` (Elbow) | Revolute | Z | $-225^\circ \dots +60^\circ$ | $[-3.9270, +1.0472]$ | $5.0\text{ rad/s}$ | $200\text{ N}\cdot\text{m}$ | $6.924\text{ kg}$ |
| **$J_4$** | `W1J` (Wrist Roll) | Revolute | Z | $-90^\circ \dots +90^\circ$ | $[-1.5708, +1.5708]$ | $5.0\text{ rad/s}$ | $200\text{ N}\cdot\text{m}$ | $1.641\text{ kg}$ |
| **$J_5$** | `W2J` (Wrist Pitch) | Revolute | Z | $-60^\circ \dots +150^\circ$ | $[-1.0472, +2.6180]$ | $5.0\text{ rad/s}$ | $200\text{ N}\cdot\text{m}$ | $2.384\text{ kg}$ |
| **$J_6$** | `W3J` (Wrist Yaw) | Revolute | Z | $-180^\circ \dots +180^\circ$ | $[-3.1416, +3.1416]$ | $5.0\text{ rad/s}$ | $200\text{ N}\cdot\text{m}$ | $2.168\text{ kg}$ |
| **$G$** | `finger_joint` | Prismatic | Z | $0 \dots 40\text{ mm}$ | $[0.000, 0.040]\text{ m}$ | $0.2\text{ m/s}$ | $50\text{ N}$ | $0.543\text{ kg}$ |

<p align="center">
  <img src="images/dh_table.png" alt="DH Parameters Table" width="720">
  <br>
  <img src="images/kinematic_chain.png" alt="Kinematic Chain Diagram" width="720">
</p>

### Forward Kinematics Zero-Pose Derivation

At zero configuration ($\theta_i = 0^\circ$ for all joints), chaining the URDF link offsets yields the precise end-effector world position:
$$\mathbf{P}_{\text{EEF}}^{\text{world}} = \begin{bmatrix} 0 + 0 + 0 + 0.0695 + 0.28625 + 0.0635 + 0.0675 \\ 0 + 0.1157 + 0 - 0.1157 + 0 + 0 + 0 \\ 0.75 + 0.1181 + 0.0775 + 0.35575 + 0 + 0.12 + 0 \end{bmatrix} = \begin{bmatrix} 0.48675\text{ m} \\ 0.00000\text{ m} \\ 1.42135\text{ m} \end{bmatrix}$$

---

## 📦 Software Stack & Package Ecosystem

```
6-dof-robotic-arm-ros1-industrial/
├── owr_description/                             # URDF, Xacro, 3D visual & collision STLs, transmissions
├── owr_gazebo/                                  # Gazebo physics worlds, spawn nodes, PID controllers
├── owr_moveit_config/                           # SRDF, joint limits, kinematics, multi-pipeline launch
├── owr_manipulation/                            # Motion orchestration (C++) and runtime safety daemon (Python)
├── owr_gripper_ikfast_arm_manipulator_plugin/   # OpenRAVE IKFast analytical solver plugin for MoveIt
├── docs/                                        # Technical reference manuals and industrial hardening checklist
└── images/                                      # Architecture, kinematic, and state-machine schematics
```

### Core Package Breakdown

| Package | Role & Contents | Key Files |
|:---|:---|:---|
| [`owr_description`](owr_description/) | Kinematic models, visual & simplified collision meshes, hardware transmission tags, Gazebo plugin bindings. | `urdf/owr.urdf.xacro`, `launch/owr_description.launch` |
| [`owr_gazebo`](owr_gazebo/) | Gazebo simulation physics, environment worlds (`pick_place.world`, `factory.world`), spawn launch files, and joint PID gain parameters. | `config/controllers.yaml`, `config/gazebo_ros_control_params.yaml`, `worlds/pick_place.world` |
| [`owr_moveit_config`](owr_moveit_config/) | MoveIt! 1.1 planning configuration, SRDF collision matrix, planning pipeline XMLs, sensor integration. | `config/owr_gripper.srdf`, `config/kinematics.yaml`, `launch/move_group.launch` |
| [`owr_manipulation`](owr_manipulation/) | C++ trajectory generation, pick-and-place state machine, dynamic planning scene manipulation, and Python safety supervisor. | `src/PickNPlace.cpp`, `src/ArmMotion.cpp`, `safety_node.py` |
| [`owr_gripper_ikfast_...`](owr_gripper_ikfast_arm_manipulator_plugin/) | High-performance C++ analytical solver plugin compiled specifically for the OWR 6-DOF kinematic tree. | `src/owr_gripper_arm_manipulator_ikfast_moveit_plugin.cpp` |

---

## ⚡ Motion Planning & Inverse Kinematics Pipeline

The motion planning pipeline translates high-level task goals into dynamically safe, collision-free joint trajectories:

```mermaid
flowchart TD
    GOAL["Motion Goal Received<br/>(Target Pose / Joint Angles / Waypoints)"] --> VALIDATE{"Workspace & Joint<br/>Limits Validation"}
    
    VALIDATE -- Out of Bounds --> REJECT["Reject Goal & Return Error Code"]
    VALIDATE -- In Bounds --> SELECTOR{"Select Planning<br/>Pipeline"}
    
    SELECTOR -- Deterministic Motion --> PILZ["Pilz Industrial Planner<br/>• LIN (Cartesian Straight)<br/>• PTP (Point-to-Point)<br/>• CIRC (Circular Blend)"]
    SELECTOR -- Cluttered / Obstacles --> OMPL["OMPL Sampling Planner<br/>• RRTConnect / RRT*<br/>• PRM* / BKPIECE"]
    SELECTOR -- Trajectory Smoothing --> OPT["Optimization Planners<br/>• CHOMP (Hamiltonian Covariant)<br/>• STOMP (Stochastic Optimization)"]

    PILZ & OMPL & OPT --> IK_SOLVE["Analytical Inverse Kinematics<br/>(IKFast 6-DOF Closed-Form Solver)"]
    
    IK_SOLVE --> TIME_PARAM["Time Parameterization<br/>(IPTP / Time-Optimal Trajectory Generation)<br/>Velocity & Acceleration Bounds Enforced"]
    
    TIME_PARAM --> COLLISION{"Dynamic Collision Check<br/>against Planning Scene Monitor<br/>(OctoMap + SRDF Collision Matrix)"}
    
    COLLISION -- Collision Detected --> REPLAN{"Replanning Permitted?"}
    REPLAN -- Yes --> SELECTOR
    REPLAN -- No --> FAIL["Abort Trajectory"]
    
    COLLISION -- Path Clear --> EXECUTE["Publish JointTrajectory to Action Server<br/>(/arm_manipulator_controller/follow_joint_trajectory)"]
```

<p align="center">
  <img src="images/moveit_pipeline.png" alt="MoveIt Motion Planning Pipeline" width="760">
</p>

### Analytical IKFast Plugin Configuration
- **Plugin Identifier:** `owr_gripper_arm_manipulator_kinematics/IKFastKinematicsPlugin`
- **Search Resolution:** $0.005\text{ rad}$
- **Solve Timeout:** $0.005\text{ s}$ ($5\text{ ms}$)
- **Advantages:** Sub-millisecond determinism without numerical singularities or seed sensitivity.

---

## 🎛 Closed-Loop Control & ROS Hardware Interface

Real-time feedback control operates in closed loop between `move_group`, `ros_control`, and the physical or simulated plant:

```mermaid
flowchart TD
    subgraph MoveGroup["MoveIt! Core"]
        TRAJ["Trajectory Generator<br/>(trajectory_msgs/JointTrajectory)"]
    end

    subgraph ROSControl["ros_control Execution Stack"]
        ACTION["FollowJointTrajectory Action Server<br/>(PositionJointInterface @ 50 Hz)"]
        PID["PID Position Feedback Loops<br/>(e(t) = q_desired - q_actual)"]
        SPAWNER["Controller Manager"]
    end

    subgraph Plant["Actuation Plant (Gazebo / Hardware)"]
        GZ_PLUGIN["gazebo_ros_control Plugin<br/>(Hardware Transmission Abstraction)"]
        MOTORS["6x NEMA 17 Steppers + RAMPS 1.4<br/>OR Gazebo Physics Joint Solvers"]
    end

    subgraph Feedback["Telemetry & State Estimation"]
        JSC["JointStateController<br/>(Publishes /joint_states @ 50 Hz)"]
        RSP["robot_state_publisher<br/>(Broadcasts TF Transform Tree)"]
        SAFETY_NODE["safety_node.py<br/>(Watchdog Monitoring @ 10 Hz)"]
    end

    TRAJ -->|Execute Goal| ACTION
    ACTION --> PID
    PID -->|PWM / Position Commands| GZ_PLUGIN
    GZ_PLUGIN --> MOTORS
    MOTORS -->|Physical Joint Position| JSC
    JSC -->|/joint_states| RSP & SAFETY_NODE
    JSC -->|Feedback q_actual| ACTION
```

<p align="center">
  <img src="images/controller_stack.png" alt="Controller Stack Diagram" width="740">
  <br>
  <img src="images/ros_interface.png" alt="ROS Interface Diagram" width="740">
</p>

### Tuned Joint PID Control Profiles

<p align="center">
  <img src="images/pid_control.png" alt="PID Tuning Response" width="680">
</p>

| Joint | Proportional ($P$) | Integral ($I$) | Derivative ($D$) | $I_{\text{clamp}}$ |
|:---|:---:|:---:|:---:|:---:|
| `BJ` | $1000.0$ | $0.0$ | $100.0$ | $1.0$ |
| `SJ` | $2000.0$ | $0.0$ | $200.0$ | $1.0$ |
| `EJ` | $1500.0$ | $0.0$ | $150.0$ | $1.0$ |
| `W1J` | $500.0$ | $0.0$ | $50.0$ | $1.0$ |
| `W2J` | $500.0$ | $0.0$ | $50.0$ | $1.0$ |
| `W3J` | $100.0$ | $0.0$ | $10.0$ | $1.0$ |

---

## 👁 Real-Time 3D Perception & OctoMap Pipeline

The system builds an adaptive 3D collision map from raw sensor point clouds:

```mermaid
flowchart LR
    CAM["RGB-D Depth Camera<br/>(Asus Xtion / Kinect Sim)"] -->|Raw PointCloud2 @ 30 Hz| RAW["/camera/point_cloud"]
    
    RAW --> PASS["PCL PassThrough Filter<br/>Bounding Box: X, Y, Z Workspace"]
    
    PASS --> VOXEL["PCL VoxelGrid Filter<br/>Downsample Leaf: 1 cm (0.01m)"]
    
    VOXEL --> SELF["Self-Collision Masking<br/>(URDF Body Envelope Subtracted via TF)"]
    
    SELF --> OCTO["OctoMap Server Node<br/>3D Occupancy Octree<br/>Resolution: 5 cm (0.05m)"]
    
    OCTO -->|/world_octomap @ 10 Hz| SCENE["MoveIt! PlanningSceneMonitor<br/>(Dynamic Collision Environment)"]
    
    SCENE --> REPLAN["Real-Time Collision Avoidance<br/>& Dynamic Replanning"]
```

<p align="center">
  <img src="images/perception_detail.png" alt="Perception Pipeline Architecture" width="760">
  <br>
  <img src="images/octomap_generated_from_processed_pointcloud_data.png" alt="OctoMap Generation Preview" width="760">
</p>

---

## 🔄 Autonomous Pick-and-Place State Machine

The autonomous pick-and-place pipeline (`owr_manipulation/src/PickNPlace.cpp`) is formalized as a deterministic finite state machine with automatic collision object attachment:

```mermaid
stateDiagram-v2
    [*] --> IDLE : System Ready
    
    IDLE --> APPROACH_TARGET : Goal Received & IK Succeeded
    APPROACH_TARGET --> DESCEND_PREGRASP : Pilz PTP Move Complete
    
    DESCEND_PREGRASP --> CLOSE_GRIPPER : Linear Cartesian Descend (LIN)
    
    CLOSE_GRIPPER --> ATTACH_OBJECT : Prismatic Jaw Clamped (Effort Threshold)
    
    ATTACH_OBJECT --> LIFT_RETREAT : Planning Scene Updated (Object Attached)
    
    LIFT_RETREAT --> TRANSPORT_PAYLOAD : Cartesian Vertical Lift (+0.15m Z)
    
    TRANSPORT_PAYLOAD --> PLACE_POSITION : Industrial Path Transit (PTP/LIN)
    
    PLACE_POSITION --> OPEN_GRIPPER : Target Coordinate Reached
    
    OPEN_GRIPPER --> DETACH_OBJECT : Prismatic Jaw Fully Opened (40mm)
    
    DETACH_OBJECT --> RETREAT_HOME : Scene Updated (Object Detached)
    
    RETREAT_HOME --> IDLE : Home Pose Restored

    APPROACH_TARGET --> FAULT_RECOVERY : Planning Failure
    DESCEND_PREGRASP --> FAULT_RECOVERY : Cartesian IK Failure
    TRANSPORT_PAYLOAD --> FAULT_RECOVERY : Collision / Trajectory Abort
    
    FAULT_RECOVERY --> RETREAT_HOME : Safe Abort Vector Executed

    state FAULT_RECOVERY {
        [*] --> StopMotion
        StopMotion --> ClearSceneDiff
        ClearSceneDiff --> OpenGripperSafe
    }
```

<p align="center">
  <img src="images/pick_place_sm.png" alt="Pick and Place State Machine" width="620">
</p>

---

## 🛡 Multi-Tiered Safety & Emergency Stop Sequence

The safety layer continuously cross-verifies joint positions against physical constraints and brokers latched emergency stops:

```mermaid
sequenceDiagram
    autonumber
    participant GZ as Gazebo / Physical Encoders
    participant JSC as Joint State Controller
    participant SN as Safety Watchdog (safety_node.py @ 10Hz)
    participant ES as /emergency_stop (Latched Topic)
    participant RC as ros_control Action Server
    participant APP as PickNPlace Orchestrator

    loop Every 20 ms (50 Hz)
        GZ->>JSC: Raw Joint Encoders (Position, Velocity, Effort)
        JSC->>SN: Broadcast /joint_states
    end

    Note over SN: Margin Check: θ_lower + 0.05rad <= θ <= θ_upper - 0.05rad

    alt Normal Operation
        SN-->>ES: /emergency_stop remains FALSE
        APP->>RC: Dispatch Trajectory Waypoint
        RC->>GZ: Actuate Motors
    else Boundary Margin Exceeded OR Operator E-Stop Triggered
        SN->>ES: Publish /emergency_stop = TRUE (Latched)
        par Cancel Motion
            ES->>RC: Cancel All Active Action Goals
            RC->>GZ: Command Instant Zero-Velocity Brake
        and Abort Task
            ES->>APP: Abort Active Pick & Place Sequence
        and Hardware Interlock
            ES->>GZ: Sever Main 12V DC Motor Power Rail
        end
        Note over SN,APP: System Held in Safe Lockout until Explicit Manual Reset
    end
```

<p align="center">
  <img src="images/safety_stack.png" alt="Multi-Tiered Safety Stack" width="760">
</p>

---

## 🔩 Physical Hardware Implementation

The simulation architecture mirrors low-cost physical maker robotics hardware:

| Hardware Subsystem | Component Specification | Role / Notes |
|:---|:---|:---|
| **Microcontroller / Interface** | Arduino Mega 2560 (ATmega2560) | Executes `rosserial_arduino` bridge or low-level G-code interpreter. |
| **Power Distribution Shield** | RAMPS 1.4 (RepRap Arduino Mega Pololu Shield) | Direct header breakout for 6x stepper drivers, limit switches, and 12V rails. |
| **Actuators (Joints 1–6)** | 6x NEMA 17 Stepper Motors (`42BYGH47`) | $1.8^\circ$ step angle ($200\text{ steps/rev}$), $0.44\text{ N}\cdot\text{m}$ holding torque. |
| **Motor Drivers** | A4988 or DRV8825 Microstepping Drivers | Configured for 1/16 or 1/32 microstepping with thermal heat sinks. |
| **End-Effector** | Robotiq 2F-140 Parallel Adaptive Gripper | Prismatic finger actuation ($0\dots 40\text{ mm}$ stroke). |
| **Power Supply** | 12 V / 5 A Regulated DC Switching PSU | Dedicated high-current rail feeding motor coils through RAMPS. |

---

## 🚀 Quickstart & Launch Recipes

<p align="center">
  <img src="images/user_workflow.png" alt="User Workflow Diagram" width="760">
</p>

### 1. Prerequisites & Environment Setup
- **OS:** Ubuntu 20.04 LTS (Focal Fossa)
- **Middleware:** ROS Noetic Desktop Full (`ros-noetic-desktop-full`)
- **Build Tool:** `catkin_make` or `catkin tools`

```bash
# Update package indices and install core dependencies
sudo apt update
sudo apt install -y python3-rosdep python3-catkin-tools git

# Initialize rosdep if not previously configured
sudo rosdep init
rosdep update
```

### 2. Workspace Setup & Compilation
```bash
# Create and navigate to catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/sam-black007/6-dof-robotic-arm-ros1-industrial.git 6dof

# Resolve package dependencies automatically
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 3. Launching the Hardened Workcell

For reliable operation, launch subsystems across dedicated terminals:

#### Terminal 1: Gazebo Physics Simulation & Robot Spawn
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch owr_gazebo robot_6dof_gazebo_spawn.launch gui:=true
```

#### Terminal 2: MoveIt! Planning Framework & RViz
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch owr_moveit_config robot_6dof_moveit_sim.launch
```

#### Terminal 3: Active Safety Watchdog Supervisor
```bash
source ~/catkin_ws/devel/setup.bash
rosrun owr_manipulation safety_node.py
```

#### Terminal 4: Automated Motion Demo / Pick-and-Place
```bash
source ~/catkin_ws/devel/setup.bash

# Run basic joint/Cartesian motion smoke test:
rosrun owr_manipulation ArmMotion

# OR run autonomous 8-stage pick-and-place pipeline:
rosrun owr_manipulation PickNPlace
```

### 4. Workcell Verification Commands
```bash
# Verify controller status:
rosservice call /controller_manager/list_controllers

# Verify joint state frequency (should report ~50 Hz):
rostopic hz /joint_states

# Verify emergency stop status:
rostopic echo /emergency_stop -n 1
```

---

## 🔄 End-to-End CI/CD & Verification Workflow

The automated software quality lifecycle is governed by GitHub Actions:

```mermaid
flowchart TD
    PUSH["Git Push / Pull Request to main"] --> RUNNER["Spin Up Ubuntu 20.04 LTS Runner"]
    
    RUNNER --> DEP_SETUP["Configure ROS Noetic Apt Repositories<br/>Install python3-rosdep & catkin tools"]
    
    DEP_SETUP --> ROSDEP["rosdep init & rosdep update<br/>Resolve all package.xml dependencies"]
    
    ROSDEP --> BUILD["Compile Workspace via catkin_make<br/>-DCMAKE_BUILD_TYPE=Release"]
    
    BUILD --> TEST{"Compilation & Linking Gate"}
    
    TEST -- Errors / Missing Headers --> NOTIFY_FAIL["Workflow Fails (Red Badge)"]
    TEST -- All 5 Packages Built Cleanly --> NOTIFY_PASS["Workflow Succeeds (Green Badge)"]
```

<p align="center">
  <img src="images/ci_pipeline.png" alt="CI Pipeline Architecture" width="580">
</p>

---

## 🔍 Troubleshooting & Diagnostics

| Symptom / Error | Probable Cause | Corrective Action |
|:---|:---|:---|
| **Controller Spawner Times Out** | Gazebo physics clock not advancing or plugin failed to load. | Ensure your GPU drivers are active. Try launching with `gui:=false` or set `export LIBGL_ALWAYS_SOFTWARE=1` in virtual environments. |
| **MoveIt: "No kinematics plugin found"** | `kinematics.yaml` pointing to missing plugin shared object. | Recompile the workspace (`catkin_make`) to ensure `owr_gripper_ikfast_arm_manipulator_plugin` is properly registered with `pluginlib`. |
| **Safety Node Trips E-Stop on Boot** | Arm spawned into Gazebo near a hard limit margin. | Check the initial spawn joint positions in `robot_6dof_gazebo_spawn.launch`. Run `rostopic pub /emergency_stop std_msgs/Bool "data: false"` to clear latched state after repositing. |
| **OctoMap Empty in RViz** | Camera point cloud topic mismatch or sensor frame transform missing. | Confirm `/camera/point_cloud` is publishing (`rostopic hz /camera/point_cloud`). Ensure RViz Fixed Frame is set to `world` or `base_link`. |
| **Trajectory Aborted: Goal Tolerance Violated** | Low PID gains leading to motor lag under gravity. | Increase the proportional gain ($P$) on `EJ` or `SJ` in `owr_gazebo/config/gazebo_ros_control_params.yaml`. |

---

## 📚 Engineering Documentation

Detailed engineering reports, math models, and industrial auditing checklists are cataloged in [`docs/`](docs/):

- 📘 **[Technical Reference Manual](docs/TECHNICAL_REFERENCE.md):** Complete mathematical reference covering kinematic chains, DH parameters, forward kinematic proofs, ROS interfaces, controller parameters, and perception filtering pipelines.
- 📋 **[ROS 1 Industrial Checklist](docs/ROS1_INDUSTRIAL_CHECKLIST.md):** Formal compliance checklist tracking code cleanliness, license validation, CI gates, and safety provisions.
- 🗺 **Additional Architecture Diagrams:**
  - [Workspace Structure Layout](images/workspace_structure.png)
  - [Controller Stack Breakdown](images/controller_stack.png)
  - [ROS 1 vs ROS 2 Hardening Tradeoffs](images/ros1_vs_ros2.png)
  - [Collision Geometry Disabled Pairs](images/collision_disabled_for_3_objects.png)

---

## 📄 License & Authorship

Distributed under the open-source **MIT License**. See [`LICENSE`](LICENSE) for full details.

- **Author & Maintainer:** [sam-black007](https://github.com/sam-black007)
- **Repository:** [6-dof-robotic-arm-ros1-industrial](https://github.com/sam-black007/6-dof-robotic-arm-ros1-industrial)

```
Copyright (c) 2026 sam-black007
```

---

<div align="center">
  <sub>Hardened with precision to turn academic robotics into robust industrial workcells.</sub>
</div>
