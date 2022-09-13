#!/bin/bash

gnome-terminal --tab -- bash -c "rosrun comanipulationpy topic2data.py $1"
gnome-terminal --tab -- bash -c "rosrun comanipulationpy run_demo.py $1"
# gnome-terminal --tab -- bash -c "rosrun comanipulationpy Node-test.py $1" #added for extra node
