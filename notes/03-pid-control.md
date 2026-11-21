# 03. PID control and anti-windup

## The controller

```
u = Kp*e + Ki*integral(e) + Kd*de/dt,     e = setpoint - measurement
```

`Kp` reacts to the current error, `Ki` removes steady-state error, and `Kd` adds damping. For motor speed loops a PI controller is usually enough. The derivative term amplifies encoder noise, and the motor's own friction already gives some damping.

## Discrete version

On a microcontroller the controller runs every `Ts` seconds, so the integral becomes a sum and the derivative a difference:

```matlab
e = w_ref - w;
integral = integral + e * Ts;
derivative = (e - e_prev) / Ts;
u = Kp*e + Ki*integral + Kd*derivative;
u = min(max(u, -V_max), V_max);        % the driver can't exceed the supply
e_prev = e;
```

Pick `Ts` at least 10 times faster than the closed-loop response you want. The example uses 1 kHz against a motor with a 16 ms time constant.

## Integrator windup

When the output is saturated (the controller asks for 20 V but the supply is 12 V), the error stays large and the integral keeps growing. Once the speed finally reaches the setpoint, the oversized integral pushes the motor past it, and the controller needs a while to "unwind".

The simplest fix is conditional integration: stop adding to the integral while the output is saturated and the error would push it further into saturation.

```matlab
u_unsat = Kp*e + Ki*integral;
u = min(max(u_unsat, -V_max), V_max);
saturated = (u ~= u_unsat) && (sign(e) == sign(u_unsat));
if ~saturated
    integral = integral + e * Ts;
end
```

Other common methods are back-calculation (feed `u - u_unsat` back into the integrator through a gain) and simply clamping the integral to a range.

## Result from the example

Motor from [02-dc-motor.md](02-dc-motor.md), setpoint 220 rad/s (needs about 11 V of the 12 V available), Kp = 0.05, Ki = 6, load step of 0.008 N·m at 0.3 s:

| | Peak speed | Overshoot | Final speed |
| --- | --- | --- | --- |
| Without anti-windup | 237.6 rad/s | 8.0 % | 220.0 rad/s |
| With anti-windup | 223.7 rad/s | 1.7 % | 220.0 rad/s |

Both versions reject the load step and return to 220 rad/s, because of the integral term.

## Tuning by hand

1. Set `Ki = Kd = 0`. Raise `Kp` until the response is fast with a little overshoot.
2. Add `Ki` until the steady-state error disappears in a reasonable time. Too much `Ki` brings back overshoot and oscillation.
3. Add `Kd` only if the response still oscillates and the measurement is clean enough. Filter the derivative if it's noisy.

With the Control System Toolbox, `pidtune(P, 'PI')` gives a starting point from the transfer function model, and `pidTuner` opens an interactive tool.

## Script

[matlab/dc_motor_pid.m](../matlab/dc_motor_pid.m) runs the same PI controller with and without anti-windup and plots both.
