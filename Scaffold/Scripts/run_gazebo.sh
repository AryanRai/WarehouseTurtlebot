#!/bin/bash
# Launch Gazebo simulation with TurtleBot3

cd turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger

echo "=========================================="
echo "TurtleBot3 Gazebo World Selection"
echo "=========================================="
echo ""
echo "Available worlds:"
echo "  1) Empty World (simple testing)"
echo "  2) TurtleBot3 World (obstacles)"
echo "  3) TurtleBot3 House (complex indoor)"
echo "  4) TurtleBot3 Stage 1 (DQN training)"
echo "  5) TurtleBot3 Stage 2 (DQN training)"
echo "  6) TurtleBot3 Stage 3 (DQN training)"
echo "  7) TurtleBot3 Stage 4 (DQN training)"
echo ""
read -p "Select world (1-7): " choice

case $choice in
    1)
        echo "Launching Empty World..."
        ros2 launch turtlebot3_gazebo empty_world.launch.py
        ;;
    2)
        echo "Launching TurtleBot3 World..."
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
        ;;
    3)
        echo "Launching TurtleBot3 House..."
        ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
        ;;
    4)
        echo "Launching Stage 1..."
        ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
        ;;
    5)
        echo "Launching Stage 2..."
        ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage2.launch.py
        ;;
    6)
        echo "Launching Stage 3..."
        ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage3.launch.py
        ;;
    7)
        echo "Launching Stage 4..."
        ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py
        ;;
    *)
        echo "Invalid choice. Launching Empty World by default..."
        ros2 launch turtlebot3_gazebo empty_world.launch.py
        ;;
esac
