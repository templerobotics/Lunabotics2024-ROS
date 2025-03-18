#!/bin/bash
set -xe
echo "STARTING ROBOT TELEOPERATION CONTROL!"
ros2 launch robot_launch teleop.launch.py &

echo "STARTING ROBOT CAMERA LIVESTREAM FOOTAGE!"

# 1080p with minimal latency
ffmpeg -f v4l2 -input_format mjpeg -framerate 30 -video_size 1920x1080 -i /dev/video0 \
  -c:v rawvideo -pix_fmt yuv420p \
  -fflags nobuffer -flags low_delay \
  -f nut pipe:1 | \
ffplay -fflags nobuffer -framedrop -flags low_delay \
  -probesize 32 -analyzeduration 0 -sync ext \
  -x 640 -y 360 -window_title "Robot Digging Subsystem Camera" \
  -i - &

  
wait