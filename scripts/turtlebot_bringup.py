#!/usr/bin/env python3
"""
TurtleBot3 Hardware Bringup Manager
Manages hardware and camera bringup on physical TurtleBot3 via SSH
"""

import sys
import time
import subprocess
from typing import Optional

# Configuration
TURTLEBOT_IP = "10.42.0.1"
TURTLEBOT_USER = "ubuntu"
TURTLEBOT_PASSWORD = "turtlebot"
ROS_DOMAIN_ID = "29"

# Colors
RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
NC = '\033[0m'  # No Color


def print_color(text: str, color: str = NC):
    """Print colored text"""
    print(f"{color}{text}{NC}")


def check_paramiko():
    """Check if paramiko is installed"""
    try:
        import paramiko
        return True
    except ImportError:
        print_color(" paramiko library not installed!", RED)
        print("\nInstall it with:")
        print("  pip install paramiko")
        print("  or")
        print("  conda install paramiko")
        return False


def ssh_command(command: str, get_output: bool = False) -> Optional[str]:
    """Execute command via SSH using paramiko"""
    try:
        import paramiko
        
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(TURTLEBOT_IP, username=TURTLEBOT_USER, password=TURTLEBOT_PASSWORD, timeout=5)
        
        stdin, stdout, stderr = client.exec_command(command)
        
        if get_output:
            output = stdout.read().decode().strip()
            client.close()
            return output
        else:
            client.close()
            return None
            
    except Exception as e:
        print_color(f" SSH Error: {e}", RED)
        return None


def test_connection() -> bool:
    """Test SSH connection to TurtleBot"""
    print("1️⃣ Testing SSH connection...")
    result = ssh_command("echo 'Connected'", get_output=True)
    if result == "Connected":
        print_color(" Connected to TurtleBot", GREEN)
        return True
    else:
        print_color(f" Cannot connect to TurtleBot at {TURTLEBOT_IP}", RED)
        print("   Please check:")
        print("   - TurtleBot is powered on")
        print("   - Network connection is working")
        print("   - IP address is correct")
        return False


def find_turtlebot3_workspace() -> Optional[str]:
    """Find workspace containing turtlebot3_bringup"""
    print("\n2️⃣ Finding turtlebot3_bringup package...")
    
    # Common workspace locations
    workspaces = [
        "~/turtlebot3_ws",
        "~/ros2_ws", 
        "~/colcon_ws",
        "~/workspace",
        "~/dev_ws"
    ]
    
    for ws in workspaces:
        # Check if workspace exists and has turtlebot3_bringup
        result = ssh_command(
            f"bash -c '[ -f {ws}/install/setup.bash ] && source /opt/ros/jazzy/setup.bash && source {ws}/install/setup.bash && ros2 pkg list | grep turtlebot3_bringup'",
            get_output=True
        )
        if result and "turtlebot3_bringup" in result:
            print_color(f" Found turtlebot3_bringup in {ws}", GREEN)
            return ws
    
    print_color("️  Could not find turtlebot3_bringup workspace", YELLOW)
    print("   Will try system ROS only")
    return None


def sync_time():
    """Sync TurtleBot's system time with PC"""
    print("\n2️⃣ Syncing system time with TurtleBot...")
    
    # Get current time from PC
    import datetime
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"   PC time: {current_time}")
    
    # Check TurtleBot's current time
    tb_time = ssh_command("date '+%Y-%m-%d %H:%M:%S'", get_output=True)
    if tb_time:
        print(f"   TurtleBot time (before): {tb_time}")
    
    # Set TurtleBot's time to match PC
    print("   Setting TurtleBot time...")
    result = ssh_command(f"echo '{TURTLEBOT_PASSWORD}' | sudo -S date -s '{current_time}'", get_output=True)
    
    # Verify new time
    tb_time_after = ssh_command("date '+%Y-%m-%d %H:%M:%S'", get_output=True)
    if tb_time_after:
        print(f"   TurtleBot time (after): {tb_time_after}")
        print_color("    Time synchronized", GREEN)
    else:
        print_color("   ️  Could not verify time sync", YELLOW)


def start_bringup(mode: str = "both"):
    """Start hardware and/or camera bringup
    
    Args:
        mode: "robot", "camera", or "both"
    """
    print_color("\n Starting TurtleBot3 Bringup", BLUE)
    print("=" * 50)
    print(f"   IP: {TURTLEBOT_IP}")
    print(f"   User: {TURTLEBOT_USER}")
    print(f"   Domain ID: {ROS_DOMAIN_ID}")
    print(f"   Mode: {mode}")
    print()
    
    if not test_connection():
        return False
    
    # Sync time first (critical for TF timestamps)
    sync_time()
    
    # Find workspace with turtlebot3_bringup
    workspace = find_turtlebot3_workspace()
    
    # Build source command
    if workspace:
        source_cmd = f"source /opt/ros/jazzy/setup.bash && source {workspace}/install/setup.bash"
    else:
        source_cmd = "source /opt/ros/jazzy/setup.bash"
        print_color("\n️  Proceeding without workspace - this may fail", YELLOW)
    
    print("\n4️⃣ Stopping any existing bringup processes...")
    if mode in ["robot", "both"]:
        ssh_command("pkill -f 'robot.launch.py'")
    if mode in ["camera", "both"]:
        ssh_command("pkill -f 'camera.launch.py'")
    time.sleep(1)
    
    step = 5
    
    # Start robot bringup
    if mode in ["robot", "both"]:
        print(f"\n{step}️⃣ Starting hardware bringup...")
        print(f"   Command: ros2 launch turtlebot3_bringup robot.launch.py")
        print(f"   ROS_DOMAIN_ID: {ROS_DOMAIN_ID}")
        print(f"   TURTLEBOT3_MODEL: burger")
        print(f"   LDS_MODEL: LDS-01")
        
        # Explicitly set all required environment variables
        robot_cmd = f"bash -c 'source ~/.bashrc && {source_cmd} && export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && export TURTLEBOT3_MODEL=burger && export LDS_MODEL=LDS-01 && nohup ros2 launch turtlebot3_bringup robot.launch.py > /tmp/robot_bringup.log 2>&1 &'"
        ssh_command(robot_cmd)
        time.sleep(5)
        step += 1
    
    # Start camera bringup
    if mode in ["camera", "both"]:
        print(f"\n{step}️⃣ Starting camera bringup...")
        print(f"   Command: ros2 launch turtlebot3_bringup camera.launch.py")
        camera_cmd = f"bash -c 'source ~/.bashrc && {source_cmd} && export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && export TURTLEBOT3_MODEL=burger && nohup ros2 launch turtlebot3_bringup camera.launch.py > /tmp/camera_bringup.log 2>&1 &'"
        ssh_command(camera_cmd)
        time.sleep(3)
        step += 1
    
    print(f"\n{step}️⃣ Checking if processes started...")
    
    if mode in ["robot", "both"]:
        robot_count = ssh_command("pgrep -f 'robot.launch.py' | wc -l", get_output=True)
        print()
        if robot_count and int(robot_count) > 0:
            print_color(" Hardware bringup is running", GREEN)
        else:
            print_color("️  Hardware bringup may not have started", YELLOW)
            print("   Check logs: ssh ubuntu@10.42.0.1 'tail -f /tmp/robot_bringup.log'")
    
    if mode in ["camera", "both"]:
        camera_count = ssh_command("pgrep -f 'camera.launch.py' | wc -l", get_output=True)
        print()
        if camera_count and int(camera_count) > 0:
            print_color(" Camera bringup is running", GREEN)
        else:
            print_color("️  Camera bringup may not have started", YELLOW)
            print("   Check logs: ssh ubuntu@10.42.0.1 'tail -f /tmp/camera_bringup.log'")
    
    print("\n Expected topics on TurtleBot:")
    if mode in ["robot", "both"]:
        print("   Hardware: /battery_state, /cmd_vel, /imu, /joint_states, /magnetic_field,")
        print("             /odom, /robot_description, /scan, /sensor_state, /tf, /tf_static")
    if mode in ["camera", "both"]:
        print("   Camera: /image_raw, /image_raw/compressed")
    
    print("\n To check logs on TurtleBot:")
    print(f"   ssh {TURTLEBOT_USER}@{TURTLEBOT_IP}")
    if mode in ["robot", "both"]:
        print("   tail -f /tmp/robot_bringup.log")
    if mode in ["camera", "both"]:
        print("   tail -f /tmp/camera_bringup.log")
    
    print_color("\n Bringup started! Processes are running on TurtleBot.", GREEN)
    print_color("\n️  IMPORTANT: Domain ID Configuration", YELLOW)
    print(f"   The TurtleBot is now using ROS_DOMAIN_ID={ROS_DOMAIN_ID}")
    print("   Make sure your PC also uses domain 29:")
    print("   $ source scripts/ros_link_turtlebot.sh")
    
    return True


def stop_bringup(mode: str = "both"):
    """Stop hardware and/or camera bringup
    
    Args:
        mode: "robot", "camera", or "both"
    """
    print_color("\n Stopping TurtleBot3 Bringup", BLUE)
    print("=" * 50)
    print(f"   Mode: {mode}")
    print()
    
    if not test_connection():
        return False
    
    if mode in ["robot", "both"]:
        print("\n2️⃣ Stopping hardware bringup...")
        # Kill all robot-related processes
        ssh_command("pkill -9 -f 'robot.launch'; pkill -9 -f 'turtlebot3_ros'; pkill -9 -f 'hlds_laser_publisher'; pkill -9 -f 'robot_state_publisher'; killall -9 ros2 2>/dev/null")
    
    if mode in ["camera", "both"]:
        print("3️⃣ Stopping camera bringup...")
        # Kill all camera-related processes
        ssh_command("pkill -9 -f 'camera.launch'; pkill -9 -f 'camera_container'; pkill -9 -f 'v4l2_camera'")
    
    time.sleep(2)
    
    # Verify processes are stopped (check actual ROS nodes, not just launch files)
    if mode in ["robot", "both"]:
        # Check for any robot-related processes
        robot_check = ssh_command("ps aux | grep -E 'turtlebot3_ros|hlds_laser|robot_state_publisher' | grep -v grep | wc -l", get_output=True)
        if robot_check and int(robot_check) > 0:
            print_color("   ️  Some robot processes may still be running", YELLOW)
        else:
            print_color("    Hardware bringup stopped", GREEN)
    
    if mode in ["camera", "both"]:
        # Check for any camera-related processes
        camera_check = ssh_command("ps aux | grep -E 'camera_container|v4l2_camera' | grep -v grep | wc -l", get_output=True)
        if camera_check and int(camera_check) > 0:
            print_color("   ️  Some camera processes may still be running", YELLOW)
        else:
            print_color("    Camera bringup stopped", GREEN)
    
    return True


def check_status():
    """Check bringup status"""
    print_color("\n TurtleBot3 Bringup Status", BLUE)
    print("=" * 50)
    print()
    
    if not test_connection():
        return False
    
    # Check for actual ROS node processes, not just launch files
    robot_check = ssh_command("ps aux | grep -E 'turtlebot3_ros|hlds_laser|robot_state_publisher' | grep -v grep | wc -l", get_output=True)
    camera_check = ssh_command("ps aux | grep -E 'camera_container|v4l2_camera' | grep -v grep | wc -l", get_output=True)
    
    # Also check if topics are being published
    topics_check = ssh_command("ros2 topic list 2>/dev/null | grep -E '/scan|/odom|/imu' | wc -l", get_output=True)
    
    print("\nHardware Bringup:")
    if robot_check and int(robot_check) > 0:
        print_color("    Running", GREEN)
        if topics_check and int(topics_check) >= 3:
            print_color("    Publishing topics (/scan, /odom, /imu)", GREEN)
        else:
            print_color("   ️  Some topics may not be publishing", YELLOW)
    else:
        print_color("    Not running", RED)
    
    print("\nCamera Bringup:")
    if camera_check and int(camera_check) > 0:
        print_color("    Running", GREEN)
    else:
        print_color("    Not running", RED)
    
    print("\n To view logs:")
    print(f"   ssh {TURTLEBOT_USER}@{TURTLEBOT_IP}")
    print("   tail -f /tmp/robot_bringup.log")
    print("   tail -f /tmp/camera_bringup.log")
    
    return True


def show_usage():
    """Show usage information"""
    print_color("\n TurtleBot3 Hardware Bringup Manager", BLUE)
    print("=" * 50)
    print("\nUsage: python3 scripts/turtlebot_bringup.py [command] [mode]")
    print("\nCommands:")
    print("  start [mode]  - Start bringup on TurtleBot")
    print("  stop [mode]   - Stop bringup processes on TurtleBot")
    print("  status        - Check if bringup is running")
    print("\nModes (optional for start/stop):")
    print("  robot   - Only hardware bringup (robot.launch.py)")
    print("  camera  - Only camera bringup (camera.launch.py)")
    print("  both    - Both hardware and camera (default)")
    print("\nExamples:")
    print("  python3 scripts/turtlebot_bringup.py start")
    print("  python3 scripts/turtlebot_bringup.py start robot")
    print("  python3 scripts/turtlebot_bringup.py start camera")
    print("  python3 scripts/turtlebot_bringup.py stop robot")
    print("  python3 scripts/turtlebot_bringup.py status")
    print()


def main():
    """Main entry point"""
    if not check_paramiko():
        sys.exit(1)
    
    if len(sys.argv) < 2:
        show_usage()
        sys.exit(1)
    
    command = sys.argv[1].lower()
    mode = sys.argv[2].lower() if len(sys.argv) > 2 else "both"
    
    # Validate mode
    if mode not in ["robot", "camera", "both"]:
        print_color(f" Invalid mode: {mode}", RED)
        print("   Valid modes: robot, camera, both")
        sys.exit(1)
    
    if command == "start":
        success = start_bringup(mode)
    elif command == "stop":
        success = stop_bringup(mode)
    elif command == "status":
        success = check_status()
    else:
        show_usage()
        sys.exit(1)
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
