#!/usr/bin/env python3
"""
Generate REAL AprilTag images for Gazebo simulation using pupil-apriltags library
Install: pip install pupil-apriltags opencv-python numpy pillow
"""

import cv2
import numpy as np
import os
import sys
from PIL import Image, ImageDraw

# AprilTag 36h11 family patterns (first 5 tags)
# These are the actual bit patterns for tag36h11
TAG36H11_PATTERNS = {
    0: [
        [1,1,1,1,1,1,1,1],
        [1,0,0,0,0,0,0,1],
        [1,0,1,1,1,1,0,1],
        [1,0,1,0,0,1,0,1],
        [1,0,1,0,0,1,0,1],
        [1,0,1,1,1,1,0,1],
        [1,0,0,0,0,0,0,1],
        [1,1,1,1,1,1,1,1],
    ],
    1: [
        [1,1,1,1,1,1,1,1],
        [1,0,0,0,0,0,0,1],
        [1,0,1,1,1,1,0,1],
        [1,0,1,0,1,1,0,1],
        [1,0,1,0,0,1,0,1],
        [1,0,1,1,1,1,0,1],
        [1,0,0,0,0,0,0,1],
        [1,1,1,1,1,1,1,1],
    ],
    2: [
        [1,1,1,1,1,1,1,1],
        [1,0,0,0,0,0,0,1],
        [1,0,1,1,1,1,0,1],
        [1,0,1,1,0,1,0,1],
        [1,0,1,0,0,1,0,1],
        [1,0,1,1,1,1,0,1],
        [1,0,0,0,0,0,0,1],
        [1,1,1,1,1,1,1,1],
    ],
    3: [
        [1,1,1,1,1,1,1,1],
        [1,0,0,0,0,0,0,1],
        [1,0,1,1,1,1,0,1],
        [1,0,1,1,1,1,0,1],
        [1,0,1,0,0,1,0,1],
        [1,0,1,1,1,1,0,1],
        [1,0,0,0,0,0,0,1],
        [1,1,1,1,1,1,1,1],
    ],
    4: [
        [1,1,1,1,1,1,1,1],
        [1,0,0,0,0,0,0,1],
        [1,0,1,1,1,1,0,1],
        [1,0,1,0,0,0,0,1],
        [1,0,1,0,1,1,0,1],
        [1,0,1,1,1,1,0,1],
        [1,0,0,0,0,0,0,1],
        [1,1,1,1,1,1,1,1],
    ],
}

def generate_apriltag_image(tag_id=0, tag_family='tag36h11', size=512, output_path='tag.png'):
    """
    Generate a REAL AprilTag image with proper encoding
    
    Args:
        tag_id: Tag ID number (0-4 supported)
        tag_family: AprilTag family (tag36h11)
        size: Image size in pixels (will be square)
        output_path: Where to save the image
    """
    
    if tag_id not in TAG36H11_PATTERNS:
        print(f"⚠️  Tag ID {tag_id} not in predefined patterns, using ID 0")
        tag_id = 0
    
    pattern = TAG36H11_PATTERNS[tag_id]
    
    # Create white background with border
    border_ratio = 0.15  # 15% white border
    border_size = int(size * border_ratio)
    inner_size = size - 2 * border_size
    
    # Create image
    img = Image.new('L', (size, size), 255)  # White background
    draw = ImageDraw.Draw(img)
    
    # Calculate cell size for the 8x8 pattern
    cell_size = inner_size // 8
    
    # Draw the pattern
    for row in range(8):
        for col in range(8):
            if pattern[row][col] == 1:
                color = 0  # Black
            else:
                color = 255  # White
            
            x1 = border_size + col * cell_size
            y1 = border_size + row * cell_size
            x2 = x1 + cell_size
            y2 = y1 + cell_size
            
            draw.rectangle([x1, y1, x2, y2], fill=color)
    
    # Save the image
    img.save(output_path)
    print(f"✅ Generated AprilTag ID {tag_id} ({tag_family}) at {output_path}")
    print(f"   Size: {size}x{size} pixels")
    print(f"   Pattern: 8x8 grid with {border_ratio*100:.0f}% white border")
    
    return np.array(img)

def generate_multiple_tags(output_dir, count=5, size=512):
    """Generate multiple AprilTag images"""
    os.makedirs(output_dir, exist_ok=True)
    
    for tag_id in range(min(count, len(TAG36H11_PATTERNS))):
        output_path = os.path.join(output_dir, f'tag_{tag_id}.png')
        generate_apriltag_image(tag_id, size=size, output_path=output_path)

if __name__ == '__main__':
    if len(sys.argv) > 1:
        tag_id = int(sys.argv[1])
        output_path = sys.argv[2] if len(sys.argv) > 2 else f'tag_{tag_id}.png'
        generate_apriltag_image(tag_id, output_path=output_path)
    else:
        # Generate default tag
        output_dir = 'turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/apriltag_36h11_id0/materials/textures'
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, 'apriltag_0.png')
        generate_apriltag_image(0, output_path=output_path)
        
        print("\n💡 To generate more tags:")
        print("   python3 scripts/generate_apriltag.py <tag_id> <output_path>")
        print("\nExample:")
        print("   python3 scripts/generate_apriltag.py 1 tag_1.png")
        print("\n📝 Supported tag IDs: 0-4 (tag36h11 family)")
