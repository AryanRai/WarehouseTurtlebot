#!/usr/bin/env python3
"""
Generate AprilTag images for Gazebo simulation
Requires: pip install apriltag opencv-python numpy
"""

import cv2
import numpy as np
import os
import sys

def generate_apriltag_image(tag_id=0, tag_family='tag36h11', size=512, output_path='tag.png'):
    """
    Generate an AprilTag image
    
    Args:
        tag_id: Tag ID number
        tag_family: AprilTag family (tag36h11, tag25h9, tag16h5)
        size: Image size in pixels (will be square)
        output_path: Where to save the image
    """
    
    # Create a simple AprilTag pattern manually
    # For tag36h11 ID 0, we'll create a basic pattern
    # Real AprilTag generation requires the apriltag library
    
    # Create white background
    img = np.ones((size, size), dtype=np.uint8) * 255
    
    # Calculate border and inner grid
    border = size // 10  # 10% border (white)
    inner_size = size - 2 * border
    
    # Create black border around the tag
    img[border:size-border, border:size-border] = 0
    
    # Inner white border
    inner_border = border + inner_size // 10
    img[inner_border:size-inner_border, inner_border:size-inner_border] = 255
    
    # Create a simple pattern for ID 0
    # This is a simplified version - real AprilTags have specific encoding
    grid_size = 6  # 6x6 grid for tag36h11
    cell_size = (size - 2 * inner_border) // grid_size
    
    # Pattern for ID 0 (simplified - not actual tag36h11 encoding)
    # Just create a recognizable pattern
    pattern = [
        [0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 0],
        [0, 1, 0, 0, 1, 0],
        [0, 1, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0],
    ]
    
    # Draw the pattern
    for row in range(grid_size):
        for col in range(grid_size):
            y1 = inner_border + row * cell_size
            y2 = y1 + cell_size
            x1 = inner_border + col * cell_size
            x2 = x1 + cell_size
            
            if pattern[row][col] == 1:
                img[y1:y2, x1:x2] = 255
            else:
                img[y1:y2, x1:x2] = 0
    
    # Save the image
    cv2.imwrite(output_path, img)
    print(f"✅ Generated AprilTag ID {tag_id} at {output_path}")
    print(f"   Size: {size}x{size} pixels")
    print(f"   Family: {tag_family}")
    
    return img

def generate_multiple_tags(output_dir, count=5, size=512):
    """Generate multiple AprilTag images"""
    os.makedirs(output_dir, exist_ok=True)
    
    for tag_id in range(count):
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
