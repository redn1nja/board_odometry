#!/bin/bash -x

WD=$WAYLAND_DISPLAY
# Run the simulation
unset WAYLAND_DISPLAY
/usr/bin/gz sim -r worlds/iris_playpen.sdf &



pushd /home/ostap/ardupilot/


/usr/bin/konsole -e "/usr/bin/zsh -c './sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON'" &

popd

sleep 80
./bin/topics topics.txt output.txt output.avi

killall ruby
pkill xterm