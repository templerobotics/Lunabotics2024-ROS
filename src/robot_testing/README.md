# Testing how-to

## Regular
colcon test --packages-select robot_testing

## Verbose - Shows tests results in realtime & return code on failure
colcon test --packages-select robot_testing --event-handlers console_direct+ --return-code-on-test-failure

## Extrenely verbose
colcon test --packages-select robot_testing \
  --event-handlers console_direct+ \
  --return-code-on-test-failure \
  --pytest-args -v \
  --ctest-args -V

## Run in /Lunabotics2024-ROS. Puts tests results into XML file
  ./build/robot_testing/teleop_tests --gtest_output=xml:test_results.xml
