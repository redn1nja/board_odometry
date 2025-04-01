#!/bin/bash -x

# Run the simulation
/usr/bin/gz sim -r worlds/iris_playpen.sdf &



pushd /home/ostap/ardupilot/

/usr/bin/xterm -e "./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map" &

popd

sleep 60
./bin/topics topics.txt output.txt output.avi

killall ruby
pkill xterm