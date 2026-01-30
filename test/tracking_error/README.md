# Quantifying the tracking error
This test is to quantify the tracking errors of position inputs with focus on ramp (constant velocity) reference.
The test compares the performance between the JTC with position as command interface and the JTC with velocity as command interface.

## Assumption
Since the postion JTC simply forwards the postion reference to the joint it can be proven using control theory that any ramp reference will have a bounded non-zero steady state error. However when using the velocity JTC the ramp tracking error will be zero since the position error is mapped through a PID controller with feed-forward to the system which takes the velocity as command input. This way the system has one additional integrator which eliminates the ramp following error.

## Tests
Joint 1 is used for all tests. All generated ramps are below the maximum joint speed, which is constant and configured to be 1.0 rad/s.
Two ramps are configured with different speeds:

**Ramp 1**: From -1.5707 rad to 1.5707 rad in 20 s. Yielding a ramp gradient/speed ($\dot{r}_{1}$) of 0.15707 rad/s <br>
**Ramp 2**: From -1.5707 rad to 1.5707 rad in 10 s. Yielding a ramp gradient/speed of 0.31415 rad/s

The following data is collected:
*Position Setpoint* ($r_{1}$) and *Joint Position* ($q_{1}$).

The error is defined as:
$$
e_{1} = \frac{r_1-q_1}{\dot{r}_{1}}
$$

To investigate the effect of the maximum acceleration setting the test is repeated for the acceleration values given below. Note that these are the maximum hardware values to protect the joint from stalling. The final motion planning will use lower acceleration values to create trajectories.<br>
**MaxAccel 1**: 0.157 rad/s^2 <br>
**MaxAccel 2**: 1.57 rad/s^2 

### Controller Parameters
The JTC runs with an update rate of 50 Hz.
For the position test no parameters can be tuned, since the controller directly forwards the trajectory. Neither on the firmware side, since there the setPoint intern is forwarded to the stepper driver which. The stepper does **not** operate in field oriented control FOC.


### Expected Result
Tracking error stays constant relative to ramp but depends linearly on maxAccel

## Recording:

```bash
ros2 bag record -e ^\/controller_manager.* -o record_<ramp_gradient>_<maxAccel>_<note>
```
for example: here also */velocity_joint_trajectory_controller/controller_state* is recorded
```bash
ros2 bag record /controller_manager/introspection_data/full /velocity_joint_trajectory_controller/controller_state -o record_0p15707_1p57_j1_vel_p0p0_i0p0_d
0p0_kff1p0
```
recording one long ramp input is sufficient. 

After recording export to CSV:
replay topics with:
```bash
ros2 bag play record_<ramp_gradient>_<maxAccel>_<note> -p
``` 
listen and export topics:
```bash
ros2 topic echo /controller_manager/introspection_data/full --csv > record_<ramp_gradient>_<maxAccel>_<note>.csv
```

## Results
### Test 1, Position JTC
|      | 0.15707 | 0.31415 |
| ---- | ------- | ---- |
| 0.157 | ~0.52   |  ~1.0    |
| 1.57  |  ~0.07 |  ~0.12    |

weird, not what expected, error scales with ramp gradient despite normalizing it

### Test 2, Velocity JTC
Steady-state tracking error is zero, where $k_p > 0$