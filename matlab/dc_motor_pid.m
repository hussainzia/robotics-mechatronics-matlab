% DC motor speed control with a discrete PI(D) controller
%
% The controller runs at 1 kHz like it would on a microcontroller. Between
% controller updates the motor is simulated with small Euler steps.
% The script runs the same controller twice: with and without anti-windup.

% Motor parameters (same motor as dc_motor_open_loop.m)
R  = 2.0;  L = 0.5e-3;  Kt = 0.05;  Ke = 0.05;  J = 2e-5;  b = 1e-5;
V_max = 12;                 % supply limit [V]

% Controller
Kp = 0.05;                  % [V per rad/s]
Ki = 6.0;                   % [V per rad]
Kd = 0.0;                   % derivative is rarely needed for speed loops
Ts = 1e-3;                  % controller period [s]

% Simulation
dt     = 1e-5;              % plant step [s], much smaller than L/R = 0.25 ms
t_end  = 0.6;
w_set  = 220;                              % speed setpoint [rad/s], needs about 11 V
w_ref  = @(t) w_set * (t >= 0.02);
T_load = @(t) 0.008 * (t >= 0.3);          % load torque step [N*m]

results = cell(1, 2);
for anti_windup = [false, true]
    n = round(t_end / dt);
    t_log = zeros(n, 1);  w_log = zeros(n, 1);  u_log = zeros(n, 1);

    i_a = 0;  w = 0;              % motor state
    integral = 0;  e_prev = 0;    % controller memory
    u = 0;
    steps_per_update = round(Ts / dt);

    for k = 1:n
        t = (k - 1) * dt;

        % Controller update every Ts
        if mod(k - 1, steps_per_update) == 0
            e = w_ref(t) - w;
            derivative = (e - e_prev) / Ts;
            u_unsat = Kp * e + Ki * integral + Kd * derivative;
            u = min(max(u_unsat, -V_max), V_max);

            % Anti-windup: stop integrating while the output is saturated
            % and the error would push it further into saturation.
            saturated = (u ~= u_unsat) && (sign(e) == sign(u_unsat));
            if ~(anti_windup && saturated)
                integral = integral + e * Ts;
            end
            e_prev = e;
        end

        % Motor (forward Euler)
        di = (u - R * i_a - Ke * w) / L;
        dw = (Kt * i_a - b * w - T_load(t)) / J;
        i_a = i_a + di * dt;
        w   = w + dw * dt;

        t_log(k) = t;  w_log(k) = w;  u_log(k) = u;
    end
    results{anti_windup + 1} = struct('t', t_log, 'w', w_log, 'u', u_log);

    if anti_windup, label = 'with anti-windup'; else, label = 'without anti-windup'; end
    w_peak = max(w_log(t_log < 0.3));       % peak before the load step
    fprintf('%-20s  peak speed %.1f rad/s (overshoot %.1f %%), final speed %.1f rad/s\n', ...
            label, w_peak, (w_peak - w_set) / w_set * 100, w_log(end));
end

figure;
subplot(2, 1, 1);
plot(results{1}.t, results{1}.w, results{2}.t, results{2}.w, 'LineWidth', 1.5);
hold on;
plot(results{1}.t, arrayfun(w_ref, results{1}.t), 'k--');
hold off;
grid on;
ylabel('Speed [rad/s]');
legend('no anti-windup', 'anti-windup', 'setpoint', 'Location', 'southeast');
title('PI speed control, load step at 0.3 s');

subplot(2, 1, 2);
plot(results{1}.t, results{1}.u, results{2}.t, results{2}.u, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Motor voltage [V]');
