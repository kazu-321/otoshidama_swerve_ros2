#!/bin/bash
source ~/ros2_humble/install/setup.bash
source ~/otoshidama_swerve/install/setup.bash

while ! /sbin/iwconfig wlp1s0 2>&1 | grep -q "ESSID:\""; do
  sleep 1
done

sleep 3

tmux new-session -d -s otoshidama "otoshidama"