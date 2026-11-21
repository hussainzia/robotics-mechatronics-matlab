# 01. Simulation basics in MATLAB

Almost every model in these notes ends up as a set of first-order differential equations:

```
dx/dt = f(t, x, u)
```

where `x` is the state vector (positions, velocities, currents) and `u` is the input (voltage, torque, force). Once a system is written like this, MATLAB can simulate it with an ODE solver.

## Writing a model as first-order equations

A second-order equation like a mass on a spring,

```
m*x'' + c*x' + k*x = F
```

is turned into two first-order equations by choosing the state `s = [x; v]`:

```
x' = v
v' = (F - c*v - k*x) / m
```

In MATLAB this becomes an anonymous function that returns the derivative of the state:

```matlab
rhs = @(t, s) [s(2); (F - c*s(2) - k*s(1)) / m];
[t, s] = ode45(rhs, [0 6], [0; 0]);   % time span, initial state
plot(t, s(:, 1));
```

`s(:, 1)` is the position over time and `s(:, 2)` is the velocity.

## Choosing a solver

| Solver | Use it for |
| --- | --- |
| `ode45` | The default. Good for most smooth, non-stiff problems. |
| `ode23` | Loose tolerances, or slightly stiff problems. |
| `ode15s` | Stiff problems: fast and slow dynamics mixed together. |
| `ode23t` | Moderately stiff problems, mechanical systems with little damping. |

A model is "stiff" when it has time constants that differ by orders of magnitude. A DC motor is a good example: the current settles in about 0.25 ms while the speed takes about 16 ms (see [02-dc-motor.md](02-dc-motor.md)). `ode45` still works there, but it takes tiny steps. If a simulation is very slow, try `ode15s`.

## Tolerances and step size

The default tolerances (`RelTol = 1e-3`, `AbsTol = 1e-6`) are fine for plots but not for checks like energy conservation. Tighten them with `odeset`:

```matlab
opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
[t, x] = ode45(rhs, [0 5], x0, opts);
```

`MaxStep` is useful when an input changes suddenly (a step at `t = 0.15 s`, for example) and the solver might step right over it:

```matlab
opts = odeset('MaxStep', 1e-4);
```

## Fixed-step loops

Real controllers run at a fixed rate (1 kHz on a microcontroller, say), so it is often more honest to simulate with a plain `for` loop:

```matlab
dt = 1e-5;
for k = 1:n
    % controller runs every Ts seconds
    % plant is updated every dt with forward Euler
    x = x + dt * f(x, u);
end
```

Forward Euler is only stable when `dt` is well below the fastest time constant. A rule of thumb is at least 10 to 20 steps per time constant. [dc_motor_pid.m](../matlab/dc_motor_pid.m) uses this approach.

## A sanity check worth doing every time

Before trusting a simulation, check one number you can work out by hand: a static deflection, a steady-state speed, an energy that should stay constant. For the mass-spring-damper in [mass_spring_damper.m](../matlab/mass_spring_damper.m):

| Damping ratio | Theory: overshoot = exp(-pi*z/sqrt(1-z^2)) | Simulation |
| --- | --- | --- |
| 0.1 | 72.9 % | 72.7 % |
| 0.5 | 16.3 % | 16.3 % |
| 1.0 | 0 % | 0 % |

The small gap at 0.1 comes from the solver output points not landing exactly on the peak.

## Script

[matlab/mass_spring_damper.m](../matlab/mass_spring_damper.m) compares three damping ratios on the same mass and spring.
