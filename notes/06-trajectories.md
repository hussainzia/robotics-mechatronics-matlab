# 06. Trajectory planning

A trajectory says where a joint (or the tool) should be at every moment, not just where it should end up. Sending a single step to a position controller makes the motors jump, saturate and shake the robot. A smooth trajectory avoids that.

## Cubic polynomial

Four conditions (start and end position, zero start and end velocity) fix a cubic. With `s = t/T` running from 0 to 1 and `d = qf - q0`:

```
q(t)   = q0 + d*(3*s^2 - 2*s^3)
qd(t)  = d*(6*s - 6*s^2) / T
qdd(t) = d*(6 - 12*s) / T^2
```

The catch is the acceleration: it starts at `6*d/T^2` instead of zero, so the motors see a sudden torque step at the start and end of every move.

## Quintic polynomial

Adding zero start and end acceleration gives six conditions and a fifth-order polynomial:

```
q(t)   = q0 + d*(10*s^3 - 15*s^4 + 6*s^5)
qd(t)  = d*(30*s^2 - 60*s^3 + 30*s^4) / T
qdd(t) = d*(60*s - 180*s^2 + 120*s^3) / T^2
```

The acceleration now starts and ends at zero. The price is a higher peak speed.

## Trapezoidal velocity profile

Accelerate at a constant rate for `ta`, cruise at constant speed, then decelerate for `ta`. Industrial drives and most 3D printers use this kind of profile (often with the corners rounded, called S-curve). Its main advantage is that peak speed and peak acceleration can be set directly to the motor's limits.

For a move of distance `d` in time `T`:

```
v = d / (T - ta)
a = v / ta
```

`ta` has to be at most `T/2`; at exactly `T/2` there is no cruise phase and the profile becomes a triangle.

## Comparison: 0 to 90° in 2 s

| Profile | Peak speed | Peak acceleration | Jump in acceleration at start? |
| --- | --- | --- | --- |
| Cubic | 67.5 °/s | 135 °/s² | yes |
| Quintic | 84.4 °/s | 130 °/s² | no |
| Trapezoid (ta = T/4) | 60.0 °/s | 120 °/s² | yes |

The numbers follow the formulas: cubic peak speed `1.5*d/T`, quintic `1.875*d/T`.

## Joint space or Cartesian space

Planning each joint separately (joint space) is simple and the joints arrive together, but the tool follows a curve that's hard to predict. When the tool has to move in a straight line, for welding or drawing for example, plan the path in Cartesian space and convert each point to joint angles with inverse kinematics. Watch for singular poses along the way (see [04-arm-kinematics.md](04-arm-kinematics.md)).

## Script

[matlab/trajectories.m](../matlab/trajectories.m) plots position, velocity and acceleration for all three profiles. The quintic function is reused in [two_link_dynamics.m](../matlab/two_link_dynamics.m).
