#!/bin/bash
# Usage print_camera_marker_rviz.sh >> calibration_demo.rviz

# ./print_camera_marker_rviz.sh 0 1 '255; 25; 0'
# ./print_camera_marker_rviz.sh 1 1 '0; 255; 0'
# ./print_camera_marker_rviz.sh 2 1 '85; 85; 255'
# ./print_camera_marker_rviz.sh 3 1 '255; 255; 127'

for m in $(seq 0 63); do ./fun_camera_marker_rviz.sh 1 ${m} '255; 25; 0'; done
for m in $(seq 0 63); do ./fun_camera_marker_rviz.sh 2 ${m} '0; 255; 0'; done
for m in $(seq 0 63); do ./fun_camera_marker_rviz.sh 3 ${m} '85; 85; 255'; done
for m in $(seq 0 63); do ./fun_camera_marker_rviz.sh 4 ${m} '255; 255; 127'; done
