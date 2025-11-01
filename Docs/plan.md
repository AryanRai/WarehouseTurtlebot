# MTRX3760 Project 2 - Warehouse Robot DevKit Plan

## Project Scope
Develop a heterogeneous warehouse robot system using polymorphic C++ design that can emulate multiple robot types on a single TurtleBot3 platform.

## Robot Types
1. **Inspection Robot** - Camera + LiDAR + Odometry for damage detection
2. **Delivery Robot** - LiDAR + Odometry for package delivery (no camera)

## Core Requirements
- Polymorphic C++ implementation with shared functionality
- Basic motion control (differential drive)
- Virtual battery level tracking
- Data persistence (damage/delivery records to disk)
- Live demonstration capability

## Chosen Extensions (for HD)
- Autonomously build a map of the warehouse
- On request, efficiently navigate to a user-selected damage site  
- Automatically dock at charging station when low battery or job complete

## System Architecture

### UML Class Hierarchy
```
Robot (Abstract Base Class)
├── InspectionRobot
└── DeliveryRobot

FactoryManager (Factory Pattern)
├── Creates Robot instances polymorphically
└── Manages robot lifecycle

WarehouseRobotSystem (Main Orchestrator)
├── Uses FactoryManager
└── Handles user interface
```

### Key Design Patterns
- **Polymorphism**: Abstract Robot base class with virtual methods
- **Factory Pattern**: FactoryManager creates robot instances
- **State Machine**: Robot states (IDLE, MOVING, EXECUTING_TASK, etc.)

## Development Modules

### 1. [In Progress] Core System Framework
- Abstract Robot base class implementation
- Factory Manager for robot creation
- Basic state management
- Shared movement and battery functionality

### 2. [Done] SLAM & Navigation
- **Frontier-based SLAM**: Expanding Wavefront Frontier Detection for autonomous mapping
- **Path Planning**: A* algorithm with cost maps and C-space calculation
- **Pure Pursuit**: Path following with obstacle avoidance and dynamic steering
- **Map Management**: OccupancyGrid processing, map saving/loading
- **Key Components**:
  - FrontierSearch: Detects unexplored areas using breadth-first search
  - PathPlanner: A* pathfinding with hallway detection and cost optimization  
  - PurePursuit: Lookahead-based path following with real-time obstacle avoidance

### 3. [Not Started] Inspection Robot Specialization  
- Camera integration for damage detection
- Area scanning patterns
- Damage analysis and recording
- AprilTag/ArUco marker detection

### 4. [Not Started] Delivery Robot Specialization
- Delivery queue management
- Route optimization
- Package pickup/delivery simulation
- Multi-delivery scheduling

### 5. [Not Started] Battery & Charging System
- Battery level simulation
- Low battery detection
- Auto-docking at charging stations
- Charging state management

### 6. [Not Started] Integration & Testing
- ROS node integration
- System testing and validation
- Live demo preparation
- Performance optimization

## Technical Stack
- **Language**: C++ (primary), Python (ROS bindings)
- **Framework**: ROS2
- **Platform**: TurtleBot3
- **Sensors**: LiDAR, Camera, Odometry
- **Simulation**: Gazebo (optional for testing)