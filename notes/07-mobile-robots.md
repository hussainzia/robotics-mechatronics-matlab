# 07. Differential-drive mobile robots

Most small wheeled robots (line followers, robot vacuums, TurtleBot-style platforms) steer by driving two wheels at different speeds.

## Kinematics

The robot's pose is `(x, y, theta)`. With forward speed `v` and turning rate `w`:

```
x'     = v*cos(theta)
y'     = v*sin(theta)
theta' = w
```

The wheels set `v` and `w`. With wheel radius `r`, track width `b` (distance between the wheels), and wheel speeds `wR`, `wL` in rad/s:

```
v = r*(wR + wL)/2
w = r*(wR - wL)/b
```

and the other way round, which is what a controller needs:

```
wR = (2*v + w*b) / (2*r)
wL = (2*v - w*b) / (2*r)
```

The robot can't move sideways. This constraint is why parallel parking takes several moves.

## Simulating

A fixed-step loop is enough here:

```matlab
pose = pose + dt * [v*cos(pose(3)); v*sin(pose(3)); w];
```

With `dt = 0.01 s` and speeds below 1 m/s the Euler error is small. For long runs, or to match odometry from real encoders, a better update uses the exact arc for constant `v` and `w` over each step.

## Go-to-goal controller

Siegwart and Nourbakhsh (*Introduction to Autonomous Mobile Robots*) give a controller in polar coordinates that brings the robot to a point and also lines it up with a goal heading:

```
rho   = distance to the goal
alpha = angle to the goal, relative to the robot's heading
beta  = goal heading minus the direction to the goal

v = k_rho * rho
w = k_alpha * alpha + k_beta * beta
```

It's stable for `k_rho > 0`, `k_beta < 0` and `k_alpha > k_rho`. The script uses `k_rho = 0.6`, `k_alpha = 2.0`, `k_beta = -0.8`. When the goal is behind the robot, the script drives backwards instead of turning around first.

Angles must be wrapped to `[-pi, pi)`. Without that, the robot turns the long way round:

```matlab
a = mod(a + pi, 2*pi) - pi;
```

## Wheel speed limits

Real motors have a top speed. If one wheel command is over the limit, scale both wheels down by the same factor. Clipping only the fast wheel changes the ratio between them and therefore the turn radius, so the robot drives a different curve than the controller asked for.

## Result

Robot: r = 35 mm, b = 160 mm, wheel limit 15 rad/s. Goal at (1.5, 1.0) facing 90°. Stop when within 1 cm and 3°.

| Start pose | Time to goal | Heading error at stop |
| --- | --- | --- |
| (0, 0), 0° | 9.68 s | -1.1° |
| (0, 1.5), -90° | 10.50 s | -2.0° |
| (2, 0), 180° | 8.33 s | -0.7° |

## Script

[matlab/diff_drive_go_to_goal.m](../matlab/diff_drive_go_to_goal.m) drives the robot from three start poses and plots the paths.
