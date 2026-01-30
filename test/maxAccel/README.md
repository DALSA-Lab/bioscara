# Finding the maximum acceleration and deceleration values.
This test assumes no payload.
This test uses the *system_test_packages/max_accel_test/* executable to drive a joint with increasing accelerations and velocities until it can no longer follow.
The data is recorded into a CSV file.

> [!WARNING]
> The executable node is very crude and does not contain protections against crashing into joint limits. 
> Use with care!

Generally max acceleration is more subtle than max vel. in the latter the joint stops moving,
Both values can be increased with higher drive currents.

## J1
- hearable skipping when a max at 3/rad/s2
- first signs of degradation at 3rad/s2 and 1 rad/s.
- at 3rad/s2 and 2rad/s joint doesnt reach max velocity

    -> **vmax** = 1 rad/s; **amax** = 2 rad/s2

## J2
- Higher current to allow faster movement. 50%
- Max velocity at 50% current: 0.03 m/s
- At acceleration > 0.2 rad/s2 acceleration drops to 0.08 rad/s2

    -> **vmax** = 0.03 m/s; **amax** = 0.2 m/s2

## J3
- Max velocity determined to be 5 rad/s at 20% current. (run j3_15)
- max accleration at 14 rad/s2 (run j3_17)
- acc 15 rad/s2 and vel 5rad/s didnt work (run j3_18)

    -> **vmax** = 5 rad/s; **amax** = 14 rad/s2

### J4
- Max acceleration between 30 and 34 rad/s (run j4_3 and j4_4)
- Max veloctiy between 10 and 20 rad/s (run j4_5)
- cant test higher velocities as range is insufficient at given acceleration

    -> **vmax** = 10 rad/s; **amax** = 30 rad/s2
