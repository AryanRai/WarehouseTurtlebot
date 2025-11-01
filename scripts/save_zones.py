#!/usr/bin/env python3
"""
Direct zone saver - creates delivery_zones.yaml from known coordinates
Usage:
  ./scripts/save_zones.py          # Save default zones
  ./scripts/save_zones.py clear    # Clear all zones
  ./scripts/save_zones.py add x y  # Add a new zone at (x, y)
"""

import yaml
import sys
import os

output_file = 'turtlebot3_ws/delivery_zones.yaml'

def load_zones():
    """Load existing zones from file"""
    if os.path.exists(output_file):
        with open(output_file, 'r') as f:
            data = yaml.safe_load(f)
            return data.get('delivery_zones', []) if data else []
    return []

def save_zones(zones):
    """Save zones to file"""
    data = {'delivery_zones': zones}
    with open(output_file, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

def clear_zones():
    """Clear all zones"""
    save_zones([])
    print("✅ Cleared all delivery zones")
    if os.path.exists(output_file):
        os.remove(output_file)
        print(f"   Deleted {output_file}")

def add_zone(x, y, name=None):
    """Add a new zone"""
    zones = load_zones()
    if name is None:
        name = f"Zone_{len(zones) + 1}"
    
    new_zone = {
        'name': name,
        'x': float(x),
        'y': float(y),
        'z': 0.0,
        'description': 'Delivery zone added manually'
    }
    zones.append(new_zone)
    save_zones(zones)
    print(f"✅ Added {name} at ({x}, {y})")
    print(f"   Total zones: {len(zones)}")

def list_zones():
    """List all zones"""
    zones = load_zones()
    if not zones:
        print("No zones defined yet")
        return
    
    print(f"📍 {len(zones)} Delivery Zones:")
    for zone in zones:
        print(f"  • {zone['name']}: ({zone['x']:.2f}, {zone['y']:.2f})")

def save_default_zones():
    """Save default zones from exploration"""
    zones = [
        {
            'name': 'Zone_1',
            'x': 0.50,
            'y': -0.91,
            'z': 0.0,
            'description': 'Delivery zone added via RViz'
        },
        {
            'name': 'Zone_2',
            'x': -1.18,
            'y': -0.58,
            'z': 0.0,
            'description': 'Delivery zone added via RViz'
        }
    ]
    
    save_zones(zones)
    print(f"✅ Saved {len(zones)} default zones to {output_file}")
    print("\nZones:")
    for zone in zones:
        print(f"  • {zone['name']}: ({zone['x']:.2f}, {zone['y']:.2f})")

# Main
if __name__ == '__main__':
    if len(sys.argv) == 1:
        # No arguments - save default zones
        save_default_zones()
    elif sys.argv[1] == 'clear':
        clear_zones()
    elif sys.argv[1] == 'list':
        list_zones()
    elif sys.argv[1] == 'add' and len(sys.argv) >= 4:
        x, y = sys.argv[2], sys.argv[3]
        name = sys.argv[4] if len(sys.argv) > 4 else None
        add_zone(x, y, name)
    else:
        print("Usage:")
        print("  ./scripts/save_zones.py          # Save default zones")
        print("  ./scripts/save_zones.py clear    # Clear all zones")
        print("  ./scripts/save_zones.py list     # List all zones")
        print("  ./scripts/save_zones.py add x y [name]  # Add zone at (x, y)")
