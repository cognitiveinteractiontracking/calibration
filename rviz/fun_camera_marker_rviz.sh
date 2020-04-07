#!/bin/bash
# Usage print_camera_marker_rviz.sh <CAMERA_ID> <MARKER_ID> <COLOR>

# ./print_camera_marker_rviz.sh 0 1 '255; 25; 0'
# ./print_camera_marker_rviz.sh 1 1 '0; 255; 0'
# ./print_camera_marker_rviz.sh 2 1 '85; 85; 255'
# ./print_camera_marker_rviz.sh 3 1 '255; 255; 127'

CAMERA_ID=${1}
MARKER_ID=${2}
COLOR=${3}

echo -e "    - Angle Tolerance: 0.100000001
      Class: rviz/Odometry
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 255; 127
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.300000012
          Color: 204; 51; 204
          Scale: 1
          Value: true
        Value: false
      Enabled: true
      Keep: 1
      Name: cam${CAMERA_ID}_marker${MARKER_ID}
      Position Tolerance: 0.100000001
      Shape:
        Alpha: 1
        Axes Length: 1
        Axes Radius: 0.100000001
        Color: ${COLOR}
        Head Length: 0.300000012
        Head Radius: 0.100000001
        Shaft Length: 0.5
        Shaft Radius: 0.0500000007
        Value: Arrow
      Topic: /aruco3_node${CAMERA_ID}_2/cam/odom/${MARKER_ID}
      Unreliable: false
      Value: true"
