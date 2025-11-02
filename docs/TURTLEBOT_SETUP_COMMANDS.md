# TurtleBot Setup Commands - Copy/Paste Guide

## Step 1: SSH into TurtleBot

```bash
# Replace with your TurtleBot's IP
ssh ubuntu@192.168.0.XXX
# Default password: turtlebot
```

## Step 2: Install Dependencies

```bash
# Update package list
sudo apt update

# Install AprilTag libraries
sudo apt install -y ros-jazzy-apriltag ros-jazzy-apriltag-msgs

# Install OpenCV (should already be installed, but just in case)
sudo apt install -y libopencv-dev ros-jazzy-cv-bridge

# Install YAML parser
sudo apt install -y libyaml-cpp-dev

# Verify installations
dpkg -l | grep apriltag
dpkg -l | grep opencv
```

## Step 3: Create Workspace

```bash
# Create workspace on TurtleBot
mkdir -p ~/camera_ws/src
cd ~/camera_ws/src
```

## Step 4: Keep This Terminal Open

You'll copy files to this location in the next step.

---

## What's Next

After running these commands on the TurtleBot, return to your laptop to copy the camera code.
