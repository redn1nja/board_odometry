#!/bin/sh

mkdir -p ../data/test$1
mkdir -p ../output$1

./run_simulation.sh

sleep 2

mv output.avi output.txt ../data/test$1

