#!/bin/bash
# Inspection Robot Commands
# Helper script for common inspection operations

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR/../turtlebot3_ws"

# Source ROS 2 environment
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "❌ Workspace not built! Run './scripts/build_warehouse_system.sh' first."
    exit 1
fi

# Function to display usage
usage() {
    echo "Usage: $0 <command>"
    echo ""
    echo "Commands:"
    echo "  save       - Save current damage sites to file"
    echo "  start      - Start inspection operations"
    echo "  status     - Show inspection status"
    echo "  log        - View inspection log"
    echo "  sites      - List all damage sites"
    echo "  clear      - Clear all damage sites"
    echo ""
    echo "Examples:"
    echo "  $0 save     # Save sites after marking them in RViz"
    echo "  $0 start    # Begin inspections"
    echo "  $0 status   # Check current status"
    echo "  $0 log      # View inspection history"
}

# Check if command provided
if [ $# -eq 0 ]; then
    usage
    exit 1
fi

COMMAND=$1

case $COMMAND in
    save)
        echo "💾 Saving damage sites..."
        ros2 service call /save_damage_sites std_srvs/srv/Trigger
        ;;
    
    start)
        echo "🚀 Starting inspections..."
        ros2 service call /start_inspections std_srvs/srv/Trigger
        ;;
    
    status)
        echo "📊 Inspection Status:"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        ros2 topic echo /inspection/status --once
        ;;
    
    log)
        echo "📋 Inspection Log:"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        if [ -f "inspection_log.csv" ]; then
            cat inspection_log.csv | column -t -s ','
        else
            echo "No inspection log found."
        fi
        ;;
    
    sites)
        echo "📍 Damage Sites:"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        if [ -f "damage_sites.yaml" ]; then
            cat damage_sites.yaml
        else
            echo "No damage sites defined yet."
            echo "Use RViz 'Publish Point' tool to mark sites."
        fi
        ;;
    
    clear)
        echo "🗑️  Clearing all damage sites..."
        echo "⚠️  This will delete all saved sites!"
        echo -n "Are you sure? (yes/no): "
        read -r confirm
        
        if [ "$confirm" = "yes" ]; then
            rm -f damage_sites.yaml
            echo "✅ All damage sites cleared."
        else
            echo "❌ Operation cancelled."
        fi
        ;;
    
    *)
        echo "❌ Unknown command: $COMMAND"
        echo ""
        usage
        exit 1
        ;;
esac
