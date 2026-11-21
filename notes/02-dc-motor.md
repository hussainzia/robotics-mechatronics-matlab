# 02. Modelling a DC motor

The brushed DC motor shows up in almost every mechatronics project, and its model is a good example of an electrical system and a mechanical system coupled together.

## Equations

Electrical side (armature circuit):

```
V = R*i + L*di/dt + Ke*w
```

Mechanical side (rotor):

```
J*dw/dt = Kt*i - b*w - T_load
```

| Symbol | Meaning | Unit |
| --- | --- | --- |
| `V` | applied voltage | V |
| `i` | armature current | A |
| `w` | shaft speed | rad/s |
| `R`, `L` | armature resistance and inductance | ohm, H |
| `Kt` | torque constant | N·m/A |
| `Ke` | back-EMF constant | V·s/rad |
| `J` | inertia of rotor plus load | kg·m² |
| `b` | viscous friction | N·m·s/rad |

In SI units `Kt` and `Ke` have the same value. Datasheets often give `Ke` in V/krpm. To convert, multiply by `60 / (2*pi*1000)`.

With state `x = [i; w]`, the model in MATLAB is:

```matlab
rhs = @(t, s) [ (V - R*s(1) - Ke*s(2)) / L ;
                (Kt*s(1) - b*s(2) - load_torque(t)) / J ];
```

## Numbers to check before simulating

These come straight from the equations and should match the plot:

```
electrical time constant   tau_e = L / R
mechanical time constant   tau_m = J*R / (Kt*Ke + b*R)
no-load speed              w_nl  = Kt*V / (R*b + Kt*Ke)
stall current              i_stall = V / R
```

For the motor in [dc_motor_open_loop.m](../matlab/dc_motor_open_loop.m) (R = 2 ohm, L = 0.5 mH, Kt = Ke = 0.05, J = 2e-5 kg·m², b = 1e-5):

| Quantity | Value |
| --- | --- |
| tau_e | 0.25 ms |
| tau_m | 15.9 ms |
| no-load speed at 12 V | 238 rad/s (2274 rpm) |
| stall current | 6 A |

The electrical time constant is about 60 times faster than the mechanical one. That is why many textbooks drop `L` and use a first-order model `w(s)/V(s) = K / (tau_m*s + 1)`, which is fine for speed control but hides the current spike at start-up.

## Adding a load

A load torque lowers the speed and raises the current. In steady state:

```
speed drop    = T_load * R / (Kt*Ke + b*R)
current       = (b*w + T_load) / Kt
```

With 0.01 N·m of load the formula gives a drop of 7.9 rad/s. The simulation goes from 238.1 to 230.2 rad/s, and the current settles at 0.25 A.

## Transfer function form

If you have the Control System Toolbox, the same motor as a transfer function from voltage to speed is:

```matlab
s = tf('s');
P = Kt / ((J*s + b)*(L*s + R) + Kt*Ke);
step(12 * P);
```

This gives the same speed curve as the ODE version, which is a quick way to cross-check one against the other.

## Things the simple model leaves out

Coulomb (dry) friction, which stops the motor from turning at very low voltage. The voltage drop across the brushes. Heating, which raises `R` during long runs. Gearbox backlash and efficiency. Driver limits: PWM resolution, current limiting and supply sag. Add these one at a time when the simple model stops matching measurements.

## Script

[matlab/dc_motor_open_loop.m](../matlab/dc_motor_open_loop.m) applies 12 V, adds a load at 0.15 s and plots speed and current.
