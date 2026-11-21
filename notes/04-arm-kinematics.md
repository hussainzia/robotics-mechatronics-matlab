# 04. Robot arm kinematics

Kinematics is the geometry of motion: where the tool is for given joint angles (forward), and which joint angles put the tool at a given point (inverse). Forces and masses don't come in yet; that's dynamics, in [05-arm-dynamics.md](05-arm-dynamics.md).

## Rotation matrices and homogeneous transforms

A rotation by `theta` about the z-axis:

```matlab
Rz = [cos(theta), -sin(theta), 0;
      sin(theta),  cos(theta), 0;
      0,           0,          1];
```

To combine a rotation `R` and a translation `p` into one 4x4 matrix:

```matlab
T = [R, p;
     0 0 0 1];
```

Transforms chain by multiplication. If `T01` takes frame 1 to frame 0 and `T12` takes frame 2 to frame 1, then `T02 = T01 * T12`. The last column of the final matrix is the tool position.

## Denavit-Hartenberg parameters

DH parameters describe each link with four numbers:

| Parameter | Meaning |
| --- | --- |
| `a` | link length, along x_i |
| `alpha` | link twist, about x_i |
| `d` | link offset, along z_(i-1) |
| `theta` | joint angle, about z_(i-1) |

The transform for one link (standard DH convention):

```matlab
T = [cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th);
     sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a*sin(th);
     0,        sin(al),          cos(al),         d;
     0,        0,                0,               1];
```

Be careful when reading other people's tables: some books use the modified (Craig) convention, which orders the four steps differently. The two give different tables for the same robot.

For a planar two-link arm the table is short:

| Link | a | alpha | d | theta |
| --- | --- | --- | --- | --- |
| 1 | L1 | 0 | 0 | q1 |
| 2 | L2 | 0 | 0 | q2 |

## Forward kinematics of a two-link arm

```
x = L1*cos(q1) + L2*cos(q1 + q2)
y = L1*sin(q1) + L2*sin(q1 + q2)
```

With L1 = 1 m, L2 = 0.7 m and q = (30°, 45°), both the formula and the DH product give x = 1.0472 m, y = 1.1761 m.

## Inverse kinematics

For the two-link arm there is a closed-form answer. Using the law of cosines:

```
cos(q2) = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2)
q2 = atan2(±sqrt(1 - cos(q2)^2), cos(q2))
q1 = atan2(y, x) - atan2(L2*sin(q2), L1 + L2*cos(q2))
```

The `±` gives two solutions, elbow-up and elbow-down. If `|cos(q2)| > 1` the point is out of reach. Always use `atan2`, not `atan`, so the angle lands in the right quadrant.

A quick test for any IK code: feed the answer back into FK and check that you get the original point. In the script the round-trip error is around 1e-16 m.

Arms with six joints rarely have a simple formula like this. For those, numerical IK (iterating with the Jacobian) or the Robotics System Toolbox `inverseKinematics` object is the usual route.

## The Jacobian

The Jacobian maps joint speeds to tool speed: `v = J(q) * qdot`. For the two-link arm:

```matlab
J = [-L1*sin(q1) - L2*sin(q1+q2),  -L2*sin(q1+q2);
      L1*cos(q1) + L2*cos(q1+q2),   L2*cos(q1+q2)];
```

`det(J) = L1*L2*sin(q2)`. It is zero when `q2 = 0` or `180°`, the arm being fully stretched or folded. At those singular poses the tool can't move in some direction no matter how fast the joints turn, and inverting `J` blows up. Keep trajectories away from them.

The same matrix also relates forces: joint torques needed to push with force `F` at the tool are `tau = J' * F`.

## Script

[matlab/two_link_kinematics.m](../matlab/two_link_kinematics.m) does FK two ways, checks both IK solutions, computes a tool velocity with the Jacobian, and makes the arm trace a circle.
