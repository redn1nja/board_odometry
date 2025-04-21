#!/bin/bash

source /opt/ros/humble/setup.bash
cd $(dirname $0)
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j10

pushd /media/ostap/ca6f373f-511c-4a00-9b0e-7bbcb94976e6/
ros2 bag play -r 1.5 --start-offset 75 --read-ahead-queue-size 200 bag --topics /imu/data /camera/image_color /fix &> /dev/null &

popd

./build/runner_node

pkill -9 $(pgrep -f "ros2 bag play")