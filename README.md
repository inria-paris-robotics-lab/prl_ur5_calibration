prl_ur5_calibration
===

## Overview
This package provides several tools for :
1. Measuring the position of a camera wrt to a aruco marker and thus calibrating fixed camera, a.k.a **external camera calibration**
2. Calibrating the robot placement wrt to the table and the embedded camera placement wrt to the gripper, a.k.a **robot calibration**

The result of the calibrations must then be placed in [prl_ur5_robot_configuration](../prl_ur5_robot_configuration).

The image / blueprint of the calibration marker is [files/models/marker.svg](files/models/marker.svg)

## Dependencies
* tqdm
* ros-noetic-visp-hand2eye-calibration
* for robot calibration :
    * [prl_hpp_tsid](https://github.com/inria-paris-robotics-lab/prl_hpp_tsid)

# External camera calibration
## How to use
Place the aruco marker on the table (visible by the camera) and run :
```
roslaunch prl_ur5_calibration calibrate_external_camera.launch camera_name:="<camera_name>" #e.g "bravo_camera"
```
The Transform from the marker to the camera optical frame will be printed in the terminal.

# Robot calibration
This calibration gives both the pose of the camera in the tool_frame and the pose of the robot in the table frame ('left_base_link' in 'stand_link').
It works in 2 steps :
1. Generate calibration poses
2. Execute the poses to take measures and compute the calibration

Both steps read most of the **calibration parameters in [files/configuration.yaml](files/configuration.yaml)**.  
Both steps generate and read pickle files in `files/*.p`.

## Details
### Step 1
```
roslaunch prl_ur5_calibration generate_poses.launch
```
To generate the calibration poses, the script goes through the following steps.
**Note** : Each step generates a pickle file that will then be read by the next step. Each step can be skipped (for debug purposes) and the next step will read from the already existing pickle file.

1. Generate sample: sample, in a grid way, the half-sphere of poses around the calibration marker.
    (Generate [files/poses_sample.p](files/poses_sample.p) and can be skipped with `generate_samples:=False`)
2. Filter reachable: Will go through each pose generated at the previous step and will try to find robot configuration to achieve this pose. If it fails, the pose is discarded.
    (Generate [files/poses_reachable.p](files/poses_reachable.p) and can be skipped with `filter_reachable:=False`)
3. Decimate poses: Will go through the remaining poses to keep only the number set in [files/configuration.yaml](files/configuration.yaml). This step will try to keep the poses that are the furthest away from each other for better calibration.
    (Generate [files/poses_calibration.p](files/poses_calibration.p) and can be skipped with `filter_decimate:=False`)

### Step 2
```
roslaunch prl_ur5_calibration run_calibration.launch
```
This step reads the poses from [files/poses_calibration.p](files/poses_calibration.p), generated at the end of Step 1, make the arm go at each one, and take a measurement (pose between the camera and the marker). For each pose the measures are saved in [files/measures.p](files/measures.p), so if the robot/node/... crash for some reason, it is possible to re-launch it with the argument `recover:=True` to retrieve the previous measures and continue after that.

Once all the measures have been taken, the calibration is computed and the result is displayed in the terminal.
    (It is possible to skip the measurment and do only the computation with the parameter `compute_only:=True`)

## Test
The calibration can be 'test' by hand/by eye with the script [scripts/test_calibration.py](scripts/test_calibration.py) :
```
rosrun prl_ur5_calibration test_calibration.py
```
This script make the gripper go to major lines of the table to see if the gripper is properly aligned or not. (works best with closed gripper).

# Troubleshooting
## General
* Pinocchio env variable must have been sourced.
* The camera must have been launched before
* Check on the screen that the **marker estimation** is properly **drawn on top of the actual one**. This could lead to big errors otherwise!
* If the estimation is "blinking" try changing the light condition to have something stable (the algorithm won't work otherwise)

## Robot calibration
* `hppcorbaserver` and `gepetto-gui` must be runnning
* `roslaunch prl_ur5_pinocchio upload_ur5.launch` must have been launched once before.
