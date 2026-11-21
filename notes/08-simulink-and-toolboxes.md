# 08. Simulink and the MATLAB toolboxes

Every script in this repo uses plain MATLAB on purpose, so the equations stay visible. Once the basics make sense, the toolboxes save a lot of time. This is a short map of which one to use for what.

## Simulink

Block diagrams instead of code. Each block is an integrator, a gain, a sum, a saturation and so on, and the solver runs the whole diagram. It's the natural way to build control loops: plant, controller, sensor and actuator limits all sit in one picture.

The DC motor from [02-dc-motor.md](02-dc-motor.md) in Simulink: two Integrator blocks (one for current, one for speed), Gain blocks for `1/L`, `R`, `Ke`, `Kt`, `b` and `1/J`, and Sum blocks to close the loops. A PID Controller block with output saturation and anti-windup built in can be dropped in front of it.

Solver settings are under Model Settings > Solver. The same advice applies as in [01-simulation-basics.md](01-simulation-basics.md): `ode45` by default, `ode15s` or `ode23t` if the model is stiff, and a fixed-step solver if you plan to generate code for hardware.

## Simscape

Simscape models physical components instead of equations. You connect a resistor, an inductor and a DC motor block with wires, and a shaft to an inertia, and Simscape writes the equations itself. It is less error-prone for big systems, though harder to see what is going on inside.

Simscape Electrical has motors, drivers, H-bridges, batteries and sensors. Simscape Multibody handles 3D mechanisms: links, joints and contact. It can import CAD assemblies, and `smimport` brings in URDF files and exports from the CAD plug-ins.

## Control System Toolbox

Linear models and controller design.

```matlab
s = tf('s');
P = Kt / ((J*s + b)*(L*s + R) + Kt*Ke);   % motor, voltage to speed
C = pidtune(P, 'PI');                      % starting PI gains
T = feedback(C*P, 1);                      % closed loop
step(T);  bode(P);  margin(C*P);
```

`ss` builds state-space models, `lsim` simulates with any input signal, and `pidTuner` is an interactive tuning app.

## Robotics System Toolbox

Ready-made versions of what notes 04 to 07 build by hand:

```matlab
robot = loadrobot('kinovaGen3', 'DataFormat', 'column');   % built-in robot model
show(robot, homeConfiguration(robot));

ik = inverseKinematics('RigidBodyTree', robot);
target_pose = trvec2tform([0.4 0.1 0.5]);                  % 4x4 transform
weights = [0.25 0.25 0.25 1 1 1];
[q_sol, info] = ik('EndEffector_Link', target_pose, weights, homeConfiguration(robot));
```

`rigidBodyTree` builds an arm link by link, `importrobot` reads URDF files, `getTransform` does forward kinematics, and `geometricJacobian` gives the Jacobian. For dynamics, `massMatrix`, `velocityProduct`, `gravityTorque` and `forwardDynamics` compute the terms from [05-arm-dynamics.md](05-arm-dynamics.md). For mobile robots there are `differentialDriveKinematics` and `controllerPurePursuit`.

Writing the two-link versions by hand first makes these much easier to trust and debug.

## Free alternative: GNU Octave

All scripts in this repo also run in GNU Octave, which is free and mostly compatible with MATLAB syntax. Two differences matter here. Octave wants functions defined before they're used, so for the scripts with local functions, move the functions to the top and add `1;` as the first line. And Octave has no Simulink.
