#!/bin/bash
# Launch Battery Monitor Web Interface
# This script starts rosbridge and the web dashboard

export ROS_DOMAIN_ID=29

echo "🔋 Starting Battery Monitor Web Interface"
echo "=========================================="
echo ""

# Check if rosbridge is installed
if ! ros2 pkg list | grep -q "rosbridge_server"; then
    echo "❌ rosbridge_server not found!"
    echo ""
    echo "Please install it with:"
    echo "   sudo apt install ros-humble-rosbridge-server"
    exit 1
fi

# Check if node_modules exists
WEB_DIR="$(dirname "$0")/../turtlebot3_ws/src/mtrx3760_battery/web"
if [ ! -d "$WEB_DIR/node_modules" ]; then
    echo "📦 Installing web dependencies (this may take a minute)..."
    cd "$WEB_DIR"
    
    # Remove any problematic old dependencies
    rm -rf node_modules package-lock.json 2>/dev/null
    
    # Install fresh
    npm install --legacy-peer-deps
    if [ $? -ne 0 ]; then
        echo "❌ Failed to install dependencies"
        echo ""
        echo "💡 Try manually:"
        echo "   cd $WEB_DIR"
        echo "   npm install"
        exit 1
    fi
    cd - > /dev/null
    echo "✅ Dependencies installed successfully"
fi

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down Battery Monitor Web Interface..."
    
    if [ ! -z "$ROSBRIDGE_PID" ] && ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
        echo "   Stopping rosbridge..."
        kill $ROSBRIDGE_PID 2>/dev/null
    fi
    
    if [ ! -z "$WEB_PID" ] && ps -p $WEB_PID > /dev/null 2>&1; then
        echo "   Stopping web server..."
        kill $WEB_PID 2>/dev/null
    fi
    
    echo "✅ Battery Monitor Web Interface stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Source ROS workspace
cd "$(dirname "$0")/../turtlebot3_ws"
source install/setup.bash

echo "1️⃣ Starting rosbridge server..."

# Use clean launcher script to avoid conda library conflicts
SCRIPT_DIR="$(dirname "$0")"
"$SCRIPT_DIR/start_rosbridge_clean.sh" "$(pwd)" > /tmp/rosbridge.log 2>&1 &
ROSBRIDGE_PID=$!

sleep 3

if ! ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
    echo "❌ Failed to start rosbridge server!"
    echo "Check logs: tail -f /tmp/rosbridge.log"
    exit 1
fi

echo "✅ rosbridge server started (PID: $ROSBRIDGE_PID)"
echo ""

echo "2️⃣ Starting web development server..."
cd "$WEB_DIR"
npm run dev > /tmp/battery_web.log 2>&1 &
WEB_PID=$!
sleep 3

if ! ps -p $WEB_PID > /dev/null 2>&1; then
    echo "❌ Failed to start web server!"
    echo "Check logs: tail -f /tmp/battery_web.log"
    cleanup
    exit 1
fi

echo "✅ Web server started (PID: $WEB_PID)"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🎉 Battery Monitor Web Interface is ready!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📊 Open in your browser:"
echo "   http://localhost:3000"
echo ""
echo "🔌 rosbridge WebSocket:"
echo "   ws://localhost:9090"
echo ""
echo "💡 Make sure battery simulator/monitor is running:"
echo "   ros2 run mtrx3760_battery battery_simulator_node"
echo "   OR"
echo "   ros2 run mtrx3760_battery battery_monitor_node"
echo ""
echo "📝 Logs:"
echo "   rosbridge: tail -f /tmp/rosbridge.log"
echo "   web server: tail -f /tmp/battery_web.log"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Wait for user interrupt
while true; do
    if ! ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
        echo "❌ rosbridge server died!"
        break
    fi
    
    if ! ps -p $WEB_PID > /dev/null 2>&1; then
        echo "❌ Web server died!"
        break
    fi
    
    sleep 1
done

cleanup
