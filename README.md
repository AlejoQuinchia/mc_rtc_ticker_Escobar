# JVRC-1 Controller with mc_rtc_ticker

*Section 7 explains how the files should be run*

## 1. Executive Summary

This repository contains a Python controller for the **JVRC-1** robot using *mc_rtc * and *mc_rtc_ticker*.

The solution implements a state machine that executes a cyclic sequence of movements for both hands and a head orientation logic, while maintaining both feet in contact with the ground and enabling motion visualization in RViz.

### Final State of the Solution
The delivered version stably implements:

- execution with mc_rtc_ticker
- the JVRC-1 robot
- left and right foot contact with the ground
- a complete 6-state sequence:
	- LEFT_OUT
	- LEFT_BACK
	- RIGHT_OUT
	- RIGHT_BACK
	- BOTH_OUT
	- BOTH_BACK
- head motion:
	- looks toward the active hand
	- returns to the front when both hands move
- visualization in RViz using:
ros2 launch mc_rtc_ticker display.launch


### Stability Decision

During development, explicit hand orientation was tested using orientation tasks, but in this environment that introduced solver instability and unwanted behavior.

For that reason, the final stable version prioritizes:

position for the hands
orientation for the head/neck

This decision allows delivery of a reproducible, stable, and visually verifiable solution.

## 2. What this solution implements
### 2.1 State machine

The controller is modeled as a finite state machine with 6 phases:

1. LEFT_OUT
2. LEFT_BACK
3. RIGHT_OUT
4. RIGHT_BACK
5. BOTH_OUT
6. BOTH_BACK

This structure makes the controller logic:

- easy to read
- easy to debug
- easy to extend

### 2.2 Hand target positions

The following target positions are used:

Left hand: (0.5, 0.25, 1.1)
Right hand: (0.5, -0.25, 1.1)

The final stable version applies these positions through:

PositionTask("l_wrist", ...)
PositionTask("r_wrist", ...)

### 2.3 Head behavior
The gaze behavior is implemented through neck orientation using:

- OrientationTask("NECK_Y_S", ...) for yaw
- OrientationTask("NECK_P_S", ...) for pitch

## 3. Technical decisions
### 3.1 Choice of Python

Python was chosen to speed up iteration on:

- controller structure
- debugging
- integration with mc_rtc_ticker
- task composition
- experimental validation of the sequence

The priority was to build a solution that is:

- clear
- explainable
- stable
- easy to modify

### 3.2 Gaze approximation

In the Python bindings available in this environment, no dedicated task of the following type was found:

- GazeTask
- LookAtTask

For that reason, the gaze logic was implemented using neck orientation.

### 3.3 RViz handling

During development, visualization in RViz did not initially work with the original display.launch configuration.

The solution consisted of:

1. enabling the ROS plugin in mc_rtc_ticker
2. verifying the publication of topics such as:
	- /control/jvrc1/robot_description
	- /control/jvrc1/joint_states
	- /tf
3. adjusting the RViz configuration used by display.launch to make it compatible with the environment

After that adjustment, visualization became functional with:

*ros2 launch mc_rtc_ticker display.launch*

## 4. Final state: implemented vs. explored
### 4.1 Implemented and stable

- Python controller with mc_rtc_ticker
- JVRC-1 robot
- both foot contacts
- complete 6-state sequence
- target positions for both hands
- coordinated head motion
- ROS publishing and RViz visualization
- continuous repetition of the cycle

### 4.2 Explored but not kept in the final stable version

Explicit hand orientation was explored according to the exercise target:

*(w, x, y, z) = (0, 0.7, 0, 0.7)*

However, that extension produced dynamic instability in this environment.
For robustness, the final delivery was kept in its stable version.

## 5. Environment requirements

A preconfigured environment is assumed with:

- ROS 2 Humble
- mc_rtc
- mc_rtc_ticker
- JVRC-1 robot modules available
- functional RViz (open the configuration called "display_no_panel.rviz" which is properly configured)
- mc_rtc ROS plugin available and loadable

## 6. File structure
**Main controller file**

*/workspace/controllers_python/jvrc_interview/jvrc_interview.py*

**Main mc_rtc_ticker configuration**

*/workspace/controllers_python/jvrc_interview/mc_rtc_jvrc_interview.conf*

**RViz configuration used by display.launch**

The RViz configuration was adjusted to allow functional visualization of the robot in this environment.

## 7. How to run the solution
### 7.1 Launch the controller
*pkill -f mc_rtc_ticker*
*rm -f /tmp/mc_rtc_pub.ipc /tmp/mc_rtc_rep.ipc*

*source /opt/ros/humble/setup.bash*
*source /workspace/workspace/install/setup_mc_rtc.sh*
*export PYTHONPATH=/workspace/controllers_python:$PYTHONPATH*

*/workspace/workspace/install/bin/mc_rtc_ticker -f /workspace/controllers_python/jvrc_interview/mc_rtc_jvrc_interview.conf*

### 7.2 Verify ROS topics
*source /opt/ros/humble/setup.bash*
*ros2 topic list | grep -E "control|robot_description|joint|tf"*

Expected topics include:

*/control/jvrc1/robot_description*
*/control/jvrc1/joint_states*
*/tf*

### 7.3 Visualize in RViz
*source /opt/ros/humble/setup.bash*
*source /workspace/workspace/install/setup_mc_rtc.sh*
*source /workspace/workspace/devel/catkin_data_ws/install/setup.bash*
*export DISPLAY=:0*
*export LIBGL_ALWAYS_SOFTWARE=1*

*ros2 launch mc_rtc_ticker display.launch*

*/workspace/controllers_python/jvrc_interview/display_no_panel.rviz* (other opction)

## 8. Validation performed
### 8.1 Functional validation

The solution was validated by observing that:

- the controller enters a stable cycle
- the hands follow the expected sequence
- the head correctly follows the active state
- the robot remains stable in the final version

### 8.2 RViz validation

Visual validation made it possible to verify:

- left hand: outward movement and return
- right hand: outward movement and return
- both hands: outward movement and return
- head orientation:
- left
- right
- front

### 8.3 Console validation

During debugging, the following were printed:

- state changes
- evolution of current positions
- evolution of targets

This made it possible to confirm that the state machine and tasks were behaving as expected.

## 9. Known limitations
### 9.1 Full hand orientation

Full hand orientation was explored, but it was not kept in the final stable version due to solver stability issues in this environment.

### 9.2 Approximate gaze

The gaze behavior is implemented through neck orientation and not through a dedicated 3D “look-at” task.

### 9.3 Environment dependency

The solution assumes an environment where mc_rtc, mc_rtc_ticker, ROS 2, and JVRC-1 are already available.

## 10. Conclusion

The final solution delivers a stable and reproducible controller for JVRC-1 using Python, mc_rtc_ticker, and RViz, with:

- complete hand motion sequence
- both foot contacts
- functional visualization
- head logic consistent with the active hand
- a simple, clear, and extensible architecture

The central design decision was to prioritize a stable, verifiable, and technically defensible version over a more ambitious but unstable one.
