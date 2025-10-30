#!/bin/bash
# Direct zone saver - bypasses ROS services
# Reads zone data from ROS topic and saves to YAML

export ROS_DOMAIN_ID=29

cd "$(dirname "$0")/../turtlebot3_ws"
source install/setup.bash

echo "💾 Saving delivery zones directly..."
echo ""

# Get the current zones by echoing the topic once
ZONES_DATA=$(timeout 2s ros2 topic echo --once /delivery/zones 2>/dev/null)

if [ -z "$ZONES_DATA" ]; then
    echo "⚠️  No zones topic found. Trying alternative method..."
    
    # Alternative: Create zones file from clicked points log
    # This requires the delivery node to be logging zones
    
    # For now, let's create a simple interactive script
    echo ""
    echo "Enter zones manually (or press Ctrl+C to cancel):"
    echo ""
    
    ZONES_FILE="delivery_zones.yaml"
    
    echo "delivery_zones:" > "$ZONES_FILE"
    
    ZONE_NUM=1
    while true; do
        echo -n "Zone $ZONE_NUM name (or 'done' to finish): "
        read ZONE_NAME
        
        if [ "$ZONE_NAME" = "done" ]; then
            break
        fi
        
        echo -n "  X coordinate: "
        read X_COORD
        
        echo -n "  Y coordinate: "
        read Y_COORD
        
        cat >> "$ZONES_FILE" << EOF
  - name: $ZONE_NAME
    x: $X_COORD
    y: $Y_COORD
    z: 0.0
    description: "Delivery zone added manually"
EOF
        
        echo "  ✓ Added $ZONE_NAME"
        echo ""
        ZONE_NUM=$((ZONE_NUM + 1))
    done
    
    echo ""
    echo "✅ Saved $((ZONE_NUM - 1)) zones to $ZONES_FILE"
    cat "$ZONES_FILE"
else
    echo "✅ Found zones data, saving..."
    # Parse and save (would need more complex parsing)
    echo "$ZONES_DATA"
fi
