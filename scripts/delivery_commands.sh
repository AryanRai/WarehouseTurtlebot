#!/bin/bash
# Delivery Robot Helper Commands
# Quick access to common delivery robot operations

# Set ROS Domain ID to match the system
export ROS_DOMAIN_ID=29

cd "$(dirname "$0")/../turtlebot3_ws"
source install/setup.bash

case "$1" in
    save|save-zones)
        echo "💾 Saving delivery zones..."
        ros2 service call /save_delivery_zones std_srvs/srv/Trigger
        ;;
    start|start-deliveries)
        echo "🚀 Starting deliveries..."
        ros2 service call /start_deliveries std_srvs/srv/Trigger
        ;;
    status)
        echo "📊 Monitoring delivery status (Ctrl+C to stop)..."
        ros2 topic echo /delivery/status
        ;;
    log)
        echo "📋 Delivery Log:"
        echo "================"
        if [ -f "delivery_log.csv" ]; then
            cat delivery_log.csv
        else
            echo "No delivery log found yet."
        fi
        ;;
    zones)
        echo "📍 Delivery Zones:"
        echo "=================="
        if [ -f "delivery_zones.yaml" ]; then
            cat delivery_zones.yaml
        else
            echo "No zones file found yet."
            echo "Click points in RViz and run: $0 save"
        fi
        ;;
    *)
        echo "Delivery Robot Helper Commands"
        echo "=============================="
        echo ""
        echo "Usage: $0 <command>"
        echo ""
        echo "Commands:"
        echo "  save          Save delivery zones to file"
        echo "  start         Start delivery execution"
        echo "  status        Monitor delivery status (live)"
        echo "  log           View delivery log"
        echo "  zones         View saved delivery zones"
        echo ""
        echo "Examples:"
        echo "  $0 save       # After clicking zones in RViz"
        echo "  $0 start      # Begin deliveries"
        echo "  $0 status     # Watch progress"
        echo "  $0 log        # Check completed deliveries"
        ;;
esac
