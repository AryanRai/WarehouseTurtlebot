# 🎯 Complete Camera Calibration & Testing Guide

## 📍 Where Calibration Files Are Located

### **Calibration Files:**
- **HSV Calibration**: `~/hsv_calibration.yaml` (your home directory)
- **Temporary Images**: `/tmp/calibration_frame_*.jpg` and `/tmp/raw_frame_*.jpg`
- **System Logs**: Terminal output with detailed HSV values

### **Executable Location:**
- **Built Binary**: `~/MTRX3760_Project_2/turtlebot3_ws/install/warehouse_robot_system/lib/warehouse_robot_system/colour_detector_node`

---

## 🎨 **STEP-BY-STEP Calibration Process**

### **Setup (Do This First):**
```bash
# 1. SSH into TurtleBot with X11 forwarding
ssh -X ubuntu@YOUR_TURTLEBOT_IP

# 2. On TurtleBot, start the robot bringup
ros2 launch turtlebot3_bringup robot.launch.py

# 3. On your laptop, run the calibration system
cd ~/MTRX3760_Project_2
./scripts/turtlebot_camera_calibration.sh
```

### **Calibration Process:**

#### **Step 1: Choose Mode 1 (Interactive Calibration)**
```
Choose mode:
  1) 🎥 Interactive Calibration (Live GUI)  ← Choose this
```

#### **Step 2: Set Up Your Test Scene**
You need to create a test scene with:

**Required Materials:**
- 1 x AprilTag 16h5 (printed on paper)
- Green object (mould simulation) - green paper, green toy, etc.
- Blue object (water simulation) - blue paper, blue toy, etc.  
- Red object (blood simulation) - red paper, red toy, etc.

**Arrangement:** 
```
        [GREEN]
           ↑
    [RED] [TAG] [RED]
           ↓  
        [BLUE]
```

#### **Step 3: Run Calibration and Follow On-Screen Guidance**

The system will tell you:
```
🎯 CALIBRATION INSTRUCTIONS:
1. Place AprilTag 16h5 in camera view
2. Add colored patches around the tag:
   🟩 GREEN patch (mould simulation) - above tag
   🟦 BLUE patch (water simulation) - below tag  
   🟥 RED patch (blood simulation) - left/right of tag
3. Watch the colored rectangles appear around detected tag
4. Check HSV values displayed above each region
5. Press 'c' to print current values for adjustment
6. Press 's' to save final calibration
```

#### **Step 4: Press 'c' to See Current Values**
The system will print:
```
🎨 CURRENT HSV VALUES CAPTURED:
================================
📊 Current HSV Thresholds:
  🟩 GREEN (Mould):  H=35-85, S=40-255, V=40-255
  🟦 BLUE (Water):   H=90-130, S=50-255, V=50-255
  🟥 RED (Blood):    H=0-10 OR 170-180, S=50-255, V=50-255

🏷️ Tag ID 0 sampling regions:
  🟩 ABOVE (place GREEN here):  HSV [45, 180, 120]
  🟦 BELOW (place BLUE here):   HSV [110, 200, 150]
  🟥 LEFT  (place RED here):    HSV [5, 190, 140]
  🟥 RIGHT (place RED here):    HSV [175, 185, 135]

📋 NEXT STEPS:
1. Place colored objects around the AprilTag:
   • GREEN object ABOVE the tag (mould simulation)
   • BLUE object BELOW the tag (water simulation)  
   • RED objects LEFT and RIGHT of tag (blood simulation)
2. Watch the colored rectangles change in the camera window
3. When satisfied with detection, press 's' to save calibration
```

#### **Step 5: Adjust Colors and Press 's' to Save**
- Move colored objects until the HSV values look good
- Press 's' to save to `~/hsv_calibration.yaml`

---

## 🔍 **Testing AprilTag and Color Detection**

### **Test 1: Check Camera is Working**
```bash
# Choose option 4 from the calibration script
./scripts/turtlebot_camera_calibration.sh
# Select: 4) 📋 Check System Status

# Should show:
🔌 ROS2 Topics:
/camera/image_raw
/apriltag_ros/detections

📷 Camera Status:
✅ Camera publishing at X Hz

🏷️ AprilTag Status:
✅ AprilTag detection active
```

### **Test 2: Normal Detection Mode**
```bash
# Choose option 3 from the calibration script
./scripts/turtlebot_camera_calibration.sh  
# Select: 3) 🔍 Normal Detection Mode

# Place AprilTag with colored objects around it
# Watch terminal for damage detection reports:
[INFO] Detected DAMAGE_MOULD damage at tag ID 0
[INFO] Detected DAMAGE_WATER damage at tag ID 1
```

### **Test 3: Live Monitoring of Detection**
```bash
# In a separate terminal, monitor the damage reports
ros2 topic echo /warehouse/damage_reports

# You should see JSON messages like:
data: '{"tag_id":0,"damage_type":"MOULD","confidence":0.8,"timestamp":"..."}'
```

---

## 🐛 **Troubleshooting**

### **Problem: No GUI Window Opens**
**Solution:** Use Mode 2 (File-based) instead:
```bash
./scripts/turtlebot_camera_calibration.sh
# Select: 2) 📁 File-based Calibration
# Images saved to /tmp/calibration_frame_*.jpg
# View with: feh /tmp/calibration_frame_*.jpg
```

### **Problem: No AprilTag Detection**
**Check:**
```bash
# Is apriltag_ros running?
ros2 topic hz /apriltag_ros/detections

# Are you using 16h5 tags? Print one from:
# https://github.com/AprilRobotics/apriltag-imgs/tree/master/tag16h5
```

### **Problem: No Camera Data**
**Check:**
```bash
# Is camera publishing?
ros2 topic hz /camera/image_raw

# Is TurtleBot bringup running?
ps aux | grep robot.launch.py
```

### **Problem: Wrong Colors Detected**
**Solution:** 
1. Re-run calibration with better lighting
2. Use more vibrant colored objects
3. Adjust HSV ranges in the saved YAML file manually

---

## 📁 **Files Generated:**

### **~/hsv_calibration.yaml**
```yaml
# HSV Calibration for Warehouse Damage Detection
# Generated: 2025-11-01 15:30:00

green_mould:
  lower: [35, 40, 40]
  upper: [85, 255, 255]

blue_water:
  lower: [90, 50, 50]  
  upper: [130, 255, 255]

red_blood:
  lower_1: [0, 50, 50]
  upper_1: [10, 255, 255]
  lower_2: [170, 50, 50]
  upper_2: [180, 255, 255]

# Sampling parameters
sampling_offset: 20
sampling_width: 30
sampling_height: 30
```

### **Test Images in /tmp/**
- `calibration_frame_0.jpg` - First frame with overlays
- `calibration_frame_1.jpg` - Second frame with overlays
- `raw_frame_0.jpg` - Raw camera feed

---

## 🚀 **Ready to Use!**

Once calibrated, your system will:
1. **Detect AprilTags** in real-time
2. **Analyze colors** in regions around each tag
3. **Classify damage types**: Green=Mould, Blue=Water, Red=Blood
4. **Publish results** to `/warehouse/damage_reports` topic
5. **Display live feedback** in GUI or save to files

The calibration ensures accurate color detection for your specific lighting conditions and colored objects!
