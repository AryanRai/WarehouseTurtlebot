#!/usr/bin/env python3
"""
Direct zone saver - creates delivery_zones.yaml from known coordinates
"""

import yaml
import sys

# Zones you added via RViz (from the logs)
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

# Save to YAML
output_file = 'turtlebot3_ws/delivery_zones.yaml'

data = {'delivery_zones': zones}

with open(output_file, 'w') as f:
    yaml.dump(data, f, default_flow_style=False, sort_keys=False)

print(f"✅ Saved {len(zones)} zones to {output_file}")
print("\nZones:")
for zone in zones:
    print(f"  • {zone['name']}: ({zone['x']:.2f}, {zone['y']:.2f})")
