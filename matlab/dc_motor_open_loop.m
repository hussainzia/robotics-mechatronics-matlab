% Brushed DC motor: open-loop response to a 12 V step, with a load added later
%
% Electrical side:   V = R*i + L*di/dt + Ke*w
% Mechanical side:   J*dw/dt = Kt*i - b*w - T_load
%
% State vector: s = [i; w]   (current [A], speed [rad/s])

% Parameters for a small 12 V hobby motor (roughly; check your datasheet)
R  = 2.0;       % armature resistance [ohm]
L  = 0.5e-3;    % armature inductance [H]
Kt = 0.05;      % torque constant [N*m/A]
Ke = 0.05;      % back-EMF constant [V*s/rad] (equal to Kt in SI units)
J  = 2e-5;      % rotor + load inertia [kg*m^2]
b  = 1e-5;      % viscous friction [N*m*s/rad]

V      = 12;      % supply voltage [V]
T_load = 0.01;    % load torque applied at t_load [N*m]
t_load = 0.15;    % [s]
t_end  = 0.3;     % [s]

% Useful numbers to check against the plot
tau_e = L / R;                          % electrical time constant
tau_m = J * R / (Kt*Ke + b*R);          % mechanical time constant
w_nl  = Kt * V / (R*b + Kt*Ke);         % no-load speed
i_stall = V / R;                        % stall current
fprintf('Electrical time constant: %.3f ms\n', tau_e * 1000);
fprintf('Mechanical time constant: %.1f ms\n', tau_m * 1000);
fprintf('No-load speed: %.1f rad/s (%.0f rpm)\n', w_nl, w_nl * 60 / (2*pi));
fprintf('Stall current: %.1f A\n', i_stall);

load_torque = @(t) T_load * (t >= t_load);
rhs = @(t, s) [ (V - R*s(1) - Ke*s(2)) / L ;
                (Kt*s(1) - b*s(2) - load_torque(t)) / J ];

% The electrical part is much faster than the mechanical part, so limit the
% step size to keep ode45 from stepping over the start-up current spike.
opts = odeset('MaxStep', 1e-4);
[t, s] = ode45(rhs, [0 t_end], [0; 0], opts);

fprintf('Speed just before the load: %.1f rad/s\n', interp1(t, s(:, 2), t_load - 1e-3));
fprintf('Speed at the end (with load): %.1f rad/s\n', s(end, 2));
fprintf('Current at the end (with load): %.2f A\n', s(end, 1));

figure;
subplot(2, 1, 1);
plot(t, s(:, 2) * 60 / (2*pi), 'LineWidth', 1.5);
grid on;
ylabel('Speed [rpm]');
title('DC motor, 12 V step, load torque added at 0.15 s');

subplot(2, 1, 2);
plot(t, s(:, 1), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Current [A]');
