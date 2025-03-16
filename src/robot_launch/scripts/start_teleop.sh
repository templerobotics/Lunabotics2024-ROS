#!/bin/bash

set -xe

echo "STARTING ROBOT TELEOPERATION CONTROL!"

ros2 launch robot_launch teleop.launch.py &

echo "STARTING ROBOT CAMERA LIVESTREAM FOOTAGE!"

ffmpeg -f v4l2 -framerate 60 -video_size 1920x1080 -i /dev/video2 -c:v libx264 -preset ultrafast -tune zerolatency -f matroska - | ffplay -fflags nobuffer -framedrop - &
ffmpeg -f v4l2 -framerate 60 -video_size 1920x1080 -i /dev/video0 -c:v libx264 -preset ultrafast -tune zerolatency -f matroska udp://localhost:1235 &
ffmpeg -f v4l2 -framerate 60 -video_size 1920x1080 -i /dev/video1 -c:v libx264 -preset ultrafast -tune zerolatency -f matroska udp://localhost:1236 &
ffmpeg -f v4l2 -framerate 60 -video_size 1920x1080 -i /dev/video3 -c:v libx264 -preset ultrafast -tune zerolatency -f matroska udp://localhost:1237 &

wait