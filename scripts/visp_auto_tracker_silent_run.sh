#!/usr/bin/env sh

# This script allow for running visp_auto_tracker and re-route stderr on stdout
# thus preventing a lot of garbage being printed in the terminal
# (ROS is unable to log stderr in a file (even with output="log"))

args=$@
args=$(echo $args | sed 's![^ ]*$!!') # Remove the last argument, that corresponds to __log
args=$(echo $args | sed 's![^ ]*$!!') # Remove the second last argument, that corresponds to __name

rosrun visp_auto_tracker visp_auto_tracker $args 2>&1