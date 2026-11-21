% Two-link planar arm dynamics in a vertical plane
%
%   M(q)*qdd + C(q,qd) + G(q) = tau
%
% Part 1 lets the arm swing freely (tau = 0) and checks that total energy
% stays constant. That is a quick way to catch sign mistakes in M, C or G.
% Part 2 makes the arm follow a quintic trajectory, first with plain PD
% control and then with computed-torque control.
%
% Needs MATLAB R2016b or newer (local functions at the end of a script).

% Uniform rods, joint 1 at the origin, angles measured from the x-axis,
% gravity pointing along -y.
P.m1 = 1.0;  P.m2 = 0.8;          % masses [kg]
P.l1 = 1.0;  P.l2 = 0.7;          % lengths [m]
P.lc1 = P.l1 / 2;  P.lc2 = P.l2 / 2;          % centre of mass distances [m]
P.I1 = P.m1 * P.l1^2 / 12;  P.I2 = P.m2 * P.l2^2 / 12;   % inertia about COM
P.g = 9.81;

% ---- Part 1: free swing, energy check ----
x0 = [deg2rad(10); deg2rad(20); 0; 0];         % [q1; q2; qd1; qd2]
opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
[t, x] = ode45(@(t, x) arm_rhs(x, [0; 0], P), [0 5], x0, opts);

E = zeros(size(t));
for k = 1:length(t)
    E(k) = arm_energy(x(k, :)', P);
end
fprintf('Free swing: energy drift over 5 s = %.2e J (start %.4f J)\n', max(abs(E - E(1))), E(1));

figure;
subplot(2, 1, 1);
plot(t, rad2deg(x(:, 1:2)), 'LineWidth', 1.2);
grid on;
ylabel('Angle [deg]');
legend('q_1', 'q_2');
title('Free swing (no torque, no friction)');
subplot(2, 1, 2);
plot(t, E - E(1), 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Energy change [J]');

% ---- Part 2: trajectory tracking ----
q_start = [deg2rad(-90); 0];          % hanging straight down
q_goal  = [deg2rad(30); deg2rad(60)];
T_move  = 2.0;                        % duration of the move [s]
t_end   = 3.0;

Kp = diag([100, 100]);                % ~10 rad/s bandwidth
Kd = diag([20, 20]);                  % critical damping for the error dynamics

controllers = {'PD', 'computed torque'};
figure;
for c = 1:2
    ctrl = @(t, x) controller(t, x, q_start, q_goal, T_move, Kp, Kd, P, controllers{c});
    [t, x] = ode45(@(t, x) arm_rhs(x, ctrl(t, x), P), [0 t_end], [q_start; 0; 0]);

    q_des = zeros(length(t), 2);
    for k = 1:length(t)
        q_des(k, :) = quintic(t(k), q_start, q_goal, T_move)';
    end
    err = rad2deg(x(:, 1:2) - q_des);
    fprintf('%-16s max tracking error: %.3f deg, final error: (%.3f, %.3f) deg\n', ...
            controllers{c}, max(abs(err(:))), err(end, 1), err(end, 2));

    subplot(2, 1, c);
    plot(t, err, 'LineWidth', 1.2);
    grid on;
    ylabel('Error [deg]');
    title(['Tracking error, ' controllers{c}]);
    legend('q_1', 'q_2');
end
xlabel('Time [s]');


% ---------- Local functions ----------

function [M, C, G] = arm_model(q, qd, P)
    c2 = cos(q(2));  s2 = sin(q(2));
    M11 = P.m1*P.lc1^2 + P.m2*(P.l1^2 + P.lc2^2 + 2*P.l1*P.lc2*c2) + P.I1 + P.I2;
    M12 = P.m2*(P.lc2^2 + P.l1*P.lc2*c2) + P.I2;
    M22 = P.m2*P.lc2^2 + P.I2;
    M = [M11, M12; M12, M22];

    h = P.m2 * P.l1 * P.lc2 * s2;
    C = [-h * (2*qd(1)*qd(2) + qd(2)^2);
          h * qd(1)^2];

    G = [(P.m1*P.lc1 + P.m2*P.l1) * P.g * cos(q(1)) + P.m2*P.lc2*P.g*cos(q(1) + q(2));
          P.m2*P.lc2*P.g*cos(q(1) + q(2))];
end

function dx = arm_rhs(x, tau, P)
    q = x(1:2);  qd = x(3:4);
    [M, C, G] = arm_model(q, qd, P);
    qdd = M \ (tau - C - G);
    dx = [qd; qdd];
end

function E = arm_energy(x, P)
    q = x(1:2);  qd = x(3:4);
    [M, ~, ~] = arm_model(q, qd, P);
    kinetic = 0.5 * qd' * M * qd;
    potential = P.m1*P.g*P.lc1*sin(q(1)) + P.m2*P.g*(P.l1*sin(q(1)) + P.lc2*sin(q(1) + q(2)));
    E = kinetic + potential;
end

function [q, qd, qdd] = quintic(t, q0, qf, T)
    % Rest-to-rest quintic: zero velocity and acceleration at both ends
    s = min(max(t / T, 0), 1);
    d = qf - q0;
    q   = q0 + d * (10*s^3 - 15*s^4 + 6*s^5);
    qd  = d * (30*s^2 - 60*s^3 + 30*s^4) / T;
    qdd = d * (60*s - 180*s^2 + 120*s^3) / T^2;
end

function tau = controller(t, x, q0, qf, T, Kp, Kd, P, type)
    q = x(1:2);  qd = x(3:4);
    [q_des, qd_des, qdd_des] = quintic(t, q0, qf, T);
    e = q_des - q;
    ed = qd_des - qd;
    if strcmp(type, 'PD')
        tau = Kp * e + Kd * ed;
    else
        % Computed torque: cancel the arm's own dynamics, then apply PD on
        % the error. Works this well only when the model matches the arm.
        [M, C, G] = arm_model(q, qd, P);
        tau = M * (qdd_des + Kd * ed + Kp * e) + C + G;
    end
end
