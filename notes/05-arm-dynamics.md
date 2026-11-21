# 05. Robot arm dynamics and computed-torque control

Dynamics links joint torques to joint accelerations. Every rigid-arm model can be written in the same form:

```
M(q)*qdd + C(q, qd) + G(q) = tau
```

| Term | Meaning |
| --- | --- |
| `M(q)` | mass (inertia) matrix, symmetric and positive definite |
| `C(q, qd)` | Coriolis and centrifugal torques |
| `G(q)` | gravity torques |
| `tau` | motor torques at the joints |

To simulate, solve for the acceleration and integrate:

```matlab
qdd = M \ (tau - C - G);
dx = [qd; qdd];
```

Use `M \ b` rather than `inv(M) * b`. It's faster and more accurate.

## Two-link arm in a vertical plane

Uniform rods: masses `m1`, `m2`, lengths `l1`, `l2`, centre of mass at `lc = l/2`, inertia about the centre of mass `I = m*l^2/12`. Angles are measured from the horizontal x-axis, and gravity points down.

```
M11 = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)) + I1 + I2
M12 = m2*(lc2^2 + l1*lc2*cos(q2)) + I2
M22 = m2*lc2^2 + I2

h  = m2*l1*lc2*sin(q2)
C1 = -h*(2*qd1*qd2 + qd2^2)
C2 =  h*qd1^2

G1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2)
G2 = m2*lc2*g*cos(q1 + q2)
```

These come from the Lagrangian `L = kinetic energy - potential energy`. Deriving them once by hand (or with the Symbolic Math Toolbox) is worth doing; sign mistakes in `C` are very common.

## Checking the model with energy

With no torque and no friction, total energy has to stay constant:

```
E = 0.5*qd'*M*qd + m1*g*lc1*sin(q1) + m2*g*(l1*sin(q1) + lc2*sin(q1 + q2))
```

In the script the arm swings freely for 5 s (chaotically, as a double pendulum does) and the energy changes by 2.3e-7 J out of 3.59 J, which is solver error. A wrong sign in `C` or `G` would show up as energy steadily growing or shrinking.

## Following a trajectory

The arm starts hanging down at (-90°, 0°) and moves to (30°, 60°) in 2 s along a quintic path (see [06-trajectories.md](06-trajectories.md)).

PD control treats each joint separately:

```matlab
tau = Kp*e + Kd*ed;          % e = q_des - q,  ed = qd_des - qd
```

Computed-torque control uses the model to cancel the arm's dynamics:

```matlab
tau = M*(qdd_des + Kd*ed + Kp*e) + C + G;
```

If the model is exact, the error then obeys `e'' + Kd*e' + Kp*e = 0`, a simple linear system you can tune like a mass-spring-damper. With `Kp = 100` and `Kd = 20` it is critically damped at 10 rad/s.

| Controller | Max tracking error | Final error, joint 1 |
| --- | --- | --- |
| PD | 7.0° | -6.9° |
| Computed torque | 0.004° | 0.000° |

The PD controller ends 6.9° low on joint 1 because it has nothing to hold the arm up against gravity: it settles where `Kp*e` equals the gravity torque. Adding gravity compensation (`tau = Kp*e + Kd*ed + G(q)`) or an integral term fixes most of that.

The computed-torque result is this good only because the controller uses exactly the same model as the simulated arm. On real hardware the masses, friction and motor limits are never known exactly. A useful exercise is to give the controller a model with 20 % heavier links and see how the error changes.

## Script

[matlab/two_link_dynamics.m](../matlab/two_link_dynamics.m) runs the energy check, then the PD versus computed-torque comparison.
