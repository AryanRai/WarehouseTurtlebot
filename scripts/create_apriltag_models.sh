#!/bin/bash
# Create AprilTag models for Gazebo simulation
# Usage: ./scripts/create_apriltag_models.sh [count]

COUNT=${1:-5}  # Default to 5 tags

echo "️  Creating $COUNT AprilTag models for Gazebo..."
echo ""

MODELS_DIR="turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models"

for i in $(seq 0 $((COUNT-1))); do
    MODEL_NAME="apriltag_36h11_id${i}"
    MODEL_DIR="$MODELS_DIR/$MODEL_NAME"
    
    echo "Creating model: $MODEL_NAME"
    
    # Create directories
    mkdir -p "$MODEL_DIR/materials/scripts"
    mkdir -p "$MODEL_DIR/materials/textures"
    
    # Generate AprilTag image
    python3 scripts/generate_apriltag.py $i "$MODEL_DIR/materials/textures/apriltag_${i}.png"
    
    # Create model.config
    cat > "$MODEL_DIR/model.config" << EOF
<?xml version="1.0"?>
<model>
  <name>AprilTag 36h11 ID $i</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Warehouse Robot Team</name>
    <email>warehouse@example.com</email>
  </author>
  <description>
    AprilTag 36h11 family, ID $i for damage site marking
  </description>
</model>
EOF
    
    # Create model.sdf
    cat > "$MODEL_DIR/model.sdf" << EOF
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="$MODEL_NAME">
    <static>true</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      
      <!-- Visual element with AprilTag texture -->
      <visual name="visual">
        <geometry>
          <box>
            <size>0.162 0.162 0.001</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>0 0 0 1</emissive>
          <pbr>
            <metal>
              <albedo_map>model://$MODEL_NAME/materials/textures/apriltag_${i}.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      
      <!-- Collision (very thin) -->
      <collision name="collision">
        <geometry>
          <box>
            <size>0.162 0.162 0.001</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF
    
    # Create material script
    cat > "$MODEL_DIR/materials/scripts/apriltag.material" << EOF
material AprilTag/Tag${i}
{
  technique
  {
    pass
    {
      ambient 1.0 1.0 1.0 1.0
      diffuse 1.0 1.0 1.0 1.0
      specular 0.0 0.0 0.0 1.0
      emissive 0.0 0.0 0.0 1.0
      
      texture_unit
      {
        texture apriltag_${i}.png
        filtering trilinear
      }
    }
  }
}
EOF
    
    echo "   Created $MODEL_NAME"
    echo ""
done

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " Created $COUNT AprilTag models!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo " Models location:"
echo "   $MODELS_DIR/apriltag_36h11_id*"
echo ""
echo " To use in Gazebo world file:"
echo ""
echo "   <include>"
echo "     <uri>model://apriltag_36h11_id0</uri>"
echo "     <name>apriltag_damage_1</name>"
echo "     <pose>x y z roll pitch yaw</pose>"
echo "   </include>"
echo ""
echo " Example poses for walls (x y z roll pitch yaw):"
echo "   North wall: <pose>0 1.14 0.3 0 1.5708 3.14159</pose>"
echo "   South wall: <pose>0 -1.14 0.3 0 1.5708 0</pose>"
echo "   East wall:  <pose>1.14 0 0.3 0 1.5708 -1.5708</pose>"
echo "   West wall:  <pose>-1.14 0 0.3 0 1.5708 1.5708</pose>"
echo ""
echo "   Note: pitch=1.5708 (90°) makes tag stand upright on wall"
echo ""
