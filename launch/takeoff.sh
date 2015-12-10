#!/bin/bash

# Record
echo "Pressing Record..."
rosrun korg_nanokontrol2 kontrol_sim.py 28

sleep 1

# Play
echo "Pressing Play..."
rosrun korg_nanokontrol2 kontrol_sim.py 27

sleep 1

# Set (hover)
echo "Pressing Set (to hover)..."
rosrun korg_nanokontrol2 kontrol_sim.py 31

sleep 1

# Line Tracker to {0, 0, 1}
echo "LineTrackerYaw to {0, 0, 1}"
rosrun korg_nanokontrol2 kontrol_sim.py 24 0 0 0.25
