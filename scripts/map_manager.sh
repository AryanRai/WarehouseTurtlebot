#!/bin/bash
# Map Manager - Save, list, and load multiple maps
# Each map includes: .yaml, .pgm, zones, and pose files

MAPS_DIR="$(cd "$(dirname "$0")/../turtlebot3_ws" && pwd)/saved_maps"
WORKSPACE_DIR="$(cd "$(dirname "$0")/../turtlebot3_ws" && pwd)"

# Create maps directory if it doesn't exist
mkdir -p "$MAPS_DIR"

# Function to list available maps
list_maps() {
    echo " Available Maps:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    if [ ! -d "$MAPS_DIR" ] || [ -z "$(ls -A $MAPS_DIR 2>/dev/null)" ]; then
        echo "   No saved maps found."
        echo ""
        return 1
    fi
    
    local count=0
    for map_dir in "$MAPS_DIR"/*; do
        if [ -d "$map_dir" ]; then
            count=$((count + 1))
            local map_name=$(basename "$map_dir")
            local map_file="$map_dir/${map_name}.yaml"
            local zones_file="$map_dir/delivery_zones.yaml"
            
            echo ""
            echo "   [$count] $map_name"
            
            if [ -f "$map_file" ]; then
                echo "       ✓ Map file: ${map_name}.yaml"
            else
                echo "       ✗ Map file missing"
            fi
            
            if [ -f "$zones_file" ]; then
                local zone_count=$(grep -c "^  - name:" "$zones_file" 2>/dev/null || echo "0")
                echo "       ✓ Zones: $zone_count delivery zones"
            else
                echo "       ✗ No zones defined"
            fi
            
            if [ -f "$map_dir/info.txt" ]; then
                echo "       ️  $(cat $map_dir/info.txt)"
            fi
        fi
    done
    
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "   Total: $count map(s)"
    echo ""
    
    return 0
}

# Function to save current map
save_map() {
    local map_name="$1"
    local description="$2"
    
    if [ -z "$map_name" ]; then
        echo " Error: Map name required"
        echo "Usage: $0 save <map_name> [description]"
        return 1
    fi
    
    # Sanitize map name (remove special characters)
    map_name=$(echo "$map_name" | tr -cd '[:alnum:]_-')
    
    local save_dir="$MAPS_DIR/$map_name"
    
    echo " Saving current map as: $map_name"
    echo ""
    
    # Create directory
    mkdir -p "$save_dir"
    
    # Find the most recent map file
    local source_map=""
    if [ -f "$WORKSPACE_DIR/warehouse_map_final.yaml" ]; then
        source_map="warehouse_map_final"
    elif [ -f "$WORKSPACE_DIR/warehouse_map_complete.yaml" ]; then
        source_map="warehouse_map_complete"
    else
        echo " No map file found to save!"
        return 1
    fi
    
    # Copy map files
    echo "   Copying map files..."
    cp "$WORKSPACE_DIR/${source_map}.yaml" "$save_dir/${map_name}.yaml"
    cp "$WORKSPACE_DIR/${source_map}.pgm" "$save_dir/${map_name}.pgm"
    
    # Update image path in yaml file
    sed -i "s|image: .*|image: ${map_name}.pgm|" "$save_dir/${map_name}.yaml"
    
    # Copy pose file if exists
    if [ -f "$WORKSPACE_DIR/${source_map}_pose.txt" ]; then
        cp "$WORKSPACE_DIR/${source_map}_pose.txt" "$save_dir/${map_name}_pose.txt"
        echo "   ✓ Saved robot pose"
    fi
    
    # Copy delivery zones if exist
    if [ -f "$WORKSPACE_DIR/delivery_zones.yaml" ]; then
        cp "$WORKSPACE_DIR/delivery_zones.yaml" "$save_dir/delivery_zones.yaml"
        local zone_count=$(grep -c "^  - name:" "$save_dir/delivery_zones.yaml" 2>/dev/null || echo "0")
        echo "   ✓ Saved $zone_count delivery zones"
    fi
    
    # Save description
    if [ ! -z "$description" ]; then
        echo "$description" > "$save_dir/info.txt"
        echo "   ✓ Saved description"
    fi
    
    # Save timestamp
    date "+Created: %Y-%m-%d %H:%M:%S" >> "$save_dir/info.txt"
    
    echo ""
    echo " Map saved successfully to: $save_dir"
    echo ""
}

# Function to load a map
load_map() {
    local map_name="$1"
    
    if [ -z "$map_name" ]; then
        echo " Error: Map name required"
        echo "Usage: $0 load <map_name>"
        return 1
    fi
    
    local load_dir="$MAPS_DIR/$map_name"
    
    if [ ! -d "$load_dir" ]; then
        echo " Map not found: $map_name"
        echo ""
        list_maps
        return 1
    fi
    
    echo " Loading map: $map_name"
    echo ""
    
    # Check if map files exist
    if [ ! -f "$load_dir/${map_name}.yaml" ] || [ ! -f "$load_dir/${map_name}.pgm" ]; then
        echo " Map files incomplete!"
        return 1
    fi
    
    # Copy map files to workspace
    echo "   Copying map files..."
    cp "$load_dir/${map_name}.yaml" "$WORKSPACE_DIR/warehouse_map_final.yaml"
    cp "$load_dir/${map_name}.pgm" "$WORKSPACE_DIR/warehouse_map_final.pgm"
    
    # Update image path in yaml
    sed -i "s|image: .*|image: warehouse_map_final.pgm|" "$WORKSPACE_DIR/warehouse_map_final.yaml"
    
    # Copy pose file if exists
    if [ -f "$load_dir/${map_name}_pose.txt" ]; then
        cp "$load_dir/${map_name}_pose.txt" "$WORKSPACE_DIR/warehouse_map_final_pose.txt"
        echo "   ✓ Loaded robot pose"
    fi
    
    # Copy delivery zones if exist
    if [ -f "$load_dir/delivery_zones.yaml" ]; then
        cp "$load_dir/delivery_zones.yaml" "$WORKSPACE_DIR/delivery_zones.yaml"
        local zone_count=$(grep -c "^  - name:" "$WORKSPACE_DIR/delivery_zones.yaml" 2>/dev/null || echo "0")
        echo "   ✓ Loaded $zone_count delivery zones"
    else
        # Clear existing zones if map has none
        rm -f "$WORKSPACE_DIR/delivery_zones.yaml"
        echo "   ️  No zones for this map"
    fi
    
    echo ""
    echo " Map loaded successfully!"
    echo "   You can now run: ./scripts/run_autonomous_slam.sh -preload"
    echo ""
}

# Function to delete a map
delete_map() {
    local map_name="$1"
    
    if [ -z "$map_name" ]; then
        echo " Error: Map name required"
        echo "Usage: $0 delete <map_name>"
        return 1
    fi
    
    local delete_dir="$MAPS_DIR/$map_name"
    
    if [ ! -d "$delete_dir" ]; then
        echo " Map not found: $map_name"
        return 1
    fi
    
    echo "️  Delete map: $map_name"
    echo ""
    echo "️  WARNING: This action cannot be undone!"
    echo ""
    echo -n "Type 'yes' to confirm deletion: "
    read -r confirm
    
    if [ "$confirm" = "yes" ]; then
        rm -rf "$delete_dir"
        echo ""
        echo " Map deleted: $map_name"
    else
        echo ""
        echo " Deletion cancelled"
    fi
    echo ""
}

# Main command handler
case "$1" in
    list|ls)
        list_maps
        ;;
    save)
        save_map "$2" "$3"
        ;;
    load)
        load_map "$2"
        ;;
    delete|rm)
        delete_map "$2"
        ;;
    *)
        echo "Map Manager - Manage multiple warehouse maps"
        echo ""
        echo "Usage: $0 <command> [arguments]"
        echo ""
        echo "Commands:"
        echo "  list, ls              List all saved maps"
        echo "  save <name> [desc]    Save current map with name and optional description"
        echo "  load <name>           Load a saved map"
        echo "  delete <name>         Delete a saved map"
        echo ""
        echo "Examples:"
        echo "  $0 list"
        echo "  $0 save warehouse1 \"Main warehouse layout\""
        echo "  $0 load warehouse1"
        echo "  $0 delete old_map"
        echo ""
        exit 1
        ;;
esac
