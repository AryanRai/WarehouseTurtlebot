# 🎭 Polymorphic Warehouse Manager - MTRX3760 Project 2

## 📋 **Overview**

This implementation replaces the original `WarehouseManager.cpp/.hpp` files with a **fully polymorphic warehouse management system** that provides all the functionality of `run_autonomous_slam.sh` with dynamic runtime robot switching.

## 🏗️ **Polymorphic Architecture**

### **Base Class Pointer Design**
```cpp
// Single base class pointer manages all robot types
std::shared_ptr<WarehouseRobot> mActiveRobot;  ///< Base class pointer

// Polymorphic creation (upcast)
auto inspectionRobot = std::make_shared<InspectionRobot>();
mActiveRobot = std::static_pointer_cast<WarehouseRobot>(inspectionRobot);  // Upcast

auto deliveryRobot = std::make_shared<DeliveryRobot>();  
mActiveRobot = std::static_pointer_cast<WarehouseRobot>(deliveryRobot);    // Upcast
```

### **Virtual Function Calls**
```cpp
// Polymorphic method calls resolve to correct derived implementation
mActiveRobot->startOperations();  // → InspectionRobot::startOperations() or DeliveryRobot::startOperations()
mActiveRobot->stopOperations();   // → InspectionRobot::stopOperations() or DeliveryRobot::stopOperations()
mActiveRobot->isOperating();      // → InspectionRobot::isOperating() or DeliveryRobot::isOperating()
mActiveRobot->getType();          // → Returns RobotType::INSPECTION or RobotType::DELIVERY
```

### **Safe Downcasting**
```cpp
// Type-safe access to derived class methods
InspectionRobot* getInspectionRobot() {
    if (!mActiveRobot || mActiveRobot->getType() != RobotType::INSPECTION) {
        return nullptr;
    }
    return static_cast<InspectionRobot*>(mActiveRobot.get());  // Safe downcast
}
```

## 🚀 **New Implementation Files**

### **1. Polymorphic Warehouse Manager Node**
**File:** `src/Robot/warehouse_manager_node.cpp`

**Key Features:**
- ✅ **True polymorphism** with base class pointers
- ✅ **Virtual function calls** for unified interface
- ✅ **Runtime robot type switching** without process restarts
- ✅ **Automatic camera system management**
- ✅ **ROS2 service interface** for external control
- ✅ **Dynamic node management** with multi-threaded executor

**Polymorphic Operations:**
```cpp
// All operations work through single interface regardless of robot type
ros2 service call /warehouse/start_operations std_srvs/srv/Trigger
ros2 service call /warehouse/stop_operations std_srvs/srv/Trigger  
ros2 service call /warehouse/get_status std_srvs/srv/Trigger
```

### **2. Dynamic Launch Script**
**File:** `scripts/run_autonomous_slam_dynamic.sh`

**Replaces:** `run_autonomous_slam.sh` with polymorphic control

**Key Features:**
- 🎮 **Interactive menu system** (same UX as original)
- 🎭 **Polymorphic robot switching** at runtime
- 🗺️ **SLAM and Navigation management**
- 💾 **Map saving and loading**
- 📊 **Real-time status monitoring**
- 🧹 **Automatic cleanup** on exit

## 🎯 **Usage Comparison**

### **Old Way (Original Script):**
```bash
./scripts/run_autonomous_slam.sh
# [6] Inspection Mode  → kills all processes, launches inspection_robot_node
# [3] Delivery Mode    → kills all processes, launches delivery_robot_node
# Switching requires full process restart
```

### **New Way (Polymorphic Manager):**
```bash
./scripts/run_autonomous_slam_dynamic.sh  
# [3] Inspection Mode  → instant polymorphic switch to InspectionRobot
# [4] Delivery Mode    → instant polymorphic switch to DeliveryRobot
# [s] Start Operations → polymorphic call: mActiveRobot->startOperations()
# No process restarts - everything happens in memory!
```

## 🎭 **Polymorphic Control Flow**

### **Mode Switching:**
```
1. User selects mode → switchToMode(RobotMode::INSPECTION)
2. Stop current robot → mActiveRobot.reset()  // Polymorphic destruction
3. Create new robot → mActiveRobot = std::make_shared<InspectionRobot>()  // Upcast
4. Robot ready → Base pointer now points to InspectionRobot instance
```

### **Operation Calls:**
```
1. User starts ops → startOperationsCallback()
2. Polymorphic call → mActiveRobot->startOperations()  // Virtual function
3. Runtime resolution → InspectionRobot::startOperations() called
4. Derived implementation → Calls startInspections() internally
```

## 🔧 **Services & Parameters**

### **ROS2 Services:**
- `/warehouse/start_operations` - Start current robot's operations
- `/warehouse/stop_operations` - Stop current robot's operations  
- `/warehouse/get_status` - Get polymorphic robot status

### **ROS2 Parameters:**
- `/polymorphic_warehouse_manager/robot_mode` - Runtime mode switching
  - Values: `"inspection"`, `"delivery"`, `"manual"`, `"none"`

### **Runtime Control:**
```bash
# Switch modes instantly
ros2 param set /polymorphic_warehouse_manager robot_mode inspection
ros2 param set /polymorphic_warehouse_manager robot_mode delivery

# Control operations polymorphically  
ros2 service call /warehouse/start_operations std_srvs/srv/Trigger
ros2 service call /warehouse/get_status std_srvs/srv/Trigger
```

## 🎯 **Benefits Over Original System**

### **Performance:**
- ⚡ **Instant switching** (no process restart)
- 🧠 **Memory efficient** (single process)
- 🔄 **No initialization delay**

### **Architecture:**
- 🎭 **True polymorphism** demonstration
- 🔧 **Unified interface** for all robot types
- 📦 **Encapsulated robot management**

### **Usability:**
- 🎮 **Same familiar interface** as original script
- 📊 **Enhanced status reporting**
- 🛡️ **Better error handling**

## 🚀 **Quick Start**

1. **Build the system:**
   ```bash
   cd turtlebot3_ws
   colcon build --packages-select warehouse_robot_system
   ```

2. **Launch TurtleBot3 simulation:**
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

3. **Run the polymorphic warehouse manager:**
   ```bash
   ./scripts/run_autonomous_slam_dynamic.sh
   ```

4. **Use the interactive menu:**
   - `[1]` - Start SLAM
   - `[3]` - Switch to Inspection Mode (polymorphic)
   - `[s]` - Start operations (virtual function call)
   - `[4]` - Switch to Delivery Mode (polymorphic)
   - `[s]` - Start operations (different virtual function!)

## 🎓 **Educational Value**

This implementation demonstrates advanced C++ concepts:
- **Polymorphism** with base class pointers
- **Virtual function dispatch** at runtime
- **Safe type casting** and RTTI usage
- **RAII** and smart pointer management
- **Design patterns** (Strategy, Factory)

Perfect for understanding how polymorphism enables flexible, extensible robotics architectures!