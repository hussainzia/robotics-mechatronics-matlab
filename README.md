# Robotics and mechatronics simulation in MATLAB

Notes and small MATLAB scripts on modelling and simulating mechatronic systems and robots: DC motors, PID control, robot arm kinematics and dynamics, trajectory planning and wheeled mobile robots.

The scripts use plain MATLAB only (no toolboxes), so every equation is visible in the code. Each one prints a few numbers that can be checked by hand, and the notes show those checks.

## Contents

| # | Note | Script | What it covers |
| --- | --- | --- | --- |
| 01 | [Simulation basics](notes/01-simulation-basics.md) | [mass_spring_damper.m](matlab/mass_spring_damper.m) | State-space form, `ode45` and other solvers, tolerances, fixed-step loops |
| 02 | [DC motor model](notes/02-dc-motor.md) | [dc_motor_open_loop.m](matlab/dc_motor_open_loop.m) | Electrical and mechanical equations, time constants, load torque |
| 03 | [PID control](notes/03-pid-control.md) | [dc_motor_pid.m](matlab/dc_motor_pid.m) | Discrete PI speed control, saturation, anti-windup, tuning |
| 04 | [Arm kinematics](notes/04-arm-kinematics.md) | [two_link_kinematics.m](matlab/two_link_kinematics.m) | Transforms, DH parameters, forward and inverse kinematics, Jacobian |
| 05 | [Arm dynamics](notes/05-arm-dynamics.md) | [two_link_dynamics.m](matlab/two_link_dynamics.m) | Equations of motion, energy check, PD versus computed-torque control |
| 06 | [Trajectories](notes/06-trajectories.md) | [trajectories.m](matlab/trajectories.m) | Cubic, quintic and trapezoidal profiles |
| 07 | [Mobile robots](notes/07-mobile-robots.md) | [diff_drive_go_to_goal.m](matlab/diff_drive_go_to_goal.m) | Differential-drive kinematics, go-to-goal control, wheel limits |
| 08 | [Simulink and toolboxes](notes/08-simulink-and-toolboxes.md) | | Simulink, Simscape, Control System and Robotics System Toolboxes |

## Running the scripts

Open MATLAB in the `matlab` folder and run any script by name, for example:

```matlab
dc_motor_pid
```

The scripts with local functions at the end (`two_link_kinematics`, `two_link_dynamics`, `diff_drive_go_to_goal`) need MATLAB R2016b or newer. They also run in GNU Octave once the functions are moved to the top of the file (see note 08).

## Suggested order

Notes 01 to 03 are the mechatronics part: model a motor, then control it. Notes 04 to 06 build up a robot arm from geometry to dynamics to motion planning. Note 07 is about wheeled robots and can be read on its own after note 01.

## References

- J. J. Craig, *Introduction to Robotics: Mechanics and Control*
- M. W. Spong, S. Hutchinson, M. Vidyasagar, *Robot Modeling and Control*
- R. Siegwart, I. R. Nourbakhsh, D. Scaramuzza, *Introduction to Autonomous Mobile Robots*
- P. Corke, *Robotics, Vision and Control*
- K. Ogata, *Modern Control Engineering*
