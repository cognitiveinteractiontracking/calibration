#!/usr/bin/env bash

node=genicam_cam4
image=image_raw
board='chessboard'
size='7x9'
squaresize=0.1

rosrun camera_calibration cameracalibrator.py --pattern ${board} --size ${size} --square ${squaresize} image:=/${node}/${image} camera:=/${node}  --no-service-check