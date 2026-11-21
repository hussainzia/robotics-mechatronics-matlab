% Point-to-point trajectories for one joint: cubic, quintic and trapezoidal
%
% All three move from q0 to qf in T seconds and start and stop at rest.
% The plot shows why quintic and trapezoidal profiles are used on real
% machines: the cubic needs a jump in acceleration at the start and end.

q0 = 0;            % start angle [deg]
qf = 90;           % end angle [deg]
T  = 2.0;          % move time [s]
t  = linspace(0, T, 401);
s  = t / T;        % normalised time, 0..1
d  = qf - q0;

% Cubic: zero velocity at both ends
cub.q   = q0 + d * (3*s.^2 - 2*s.^3);
cub.qd  = d * (6*s - 6*s.^2) / T;
cub.qdd = d * (6 - 12*s) / T^2;

% Quintic: zero velocity and zero acceleration at both ends
qui.q   = q0 + d * (10*s.^3 - 15*s.^4 + 6*s.^5);
qui.qd  = d * (30*s.^2 - 60*s.^3 + 30*s.^4) / T;
qui.qdd = d * (60*s - 180*s.^2 + 120*s.^3) / T^2;

% Trapezoidal velocity: accelerate for ta, cruise, decelerate for ta
ta = T / 4;                          % acceleration time [s]
v  = d / (T - ta);                   % cruise speed so the area equals d
a  = v / ta;                         % acceleration [deg/s^2]
trap.q = zeros(size(t));  trap.qd = zeros(size(t));  trap.qdd = zeros(size(t));
for k = 1:length(t)
    tk = t(k);
    if tk < ta
        trap.qdd(k) = a;
        trap.qd(k)  = a * tk;
        trap.q(k)   = q0 + 0.5 * a * tk^2;
    elseif tk < T - ta
        trap.qd(k)  = v;
        trap.q(k)   = q0 + 0.5 * a * ta^2 + v * (tk - ta);
    else
        tr = T - tk;                 % time remaining
        trap.qdd(k) = -a;
        trap.qd(k)  = a * tr;
        trap.q(k)   = qf - 0.5 * a * tr^2;
    end
end

fprintf('Peak speed    cubic %.1f, quintic %.1f, trapezoid %.1f deg/s\n', ...
        max(cub.qd), max(qui.qd), max(trap.qd));
fprintf('Peak accel    cubic %.1f, quintic %.1f, trapezoid %.1f deg/s^2\n', ...
        max(abs(cub.qdd)), max(abs(qui.qdd)), max(abs(trap.qdd)));
fprintf('End position  cubic %.3f, quintic %.3f, trapezoid %.3f deg\n', ...
        cub.q(end), qui.q(end), trap.q(end));

figure;
names = {'q', 'qd', 'qdd'};
labels = {'Position [deg]', 'Velocity [deg/s]', 'Acceleration [deg/s^2]'};
for i = 1:3
    subplot(3, 1, i);
    plot(t, cub.(names{i}), t, qui.(names{i}), t, trap.(names{i}), 'LineWidth', 1.3);
    grid on;
    ylabel(labels{i});
end
xlabel('Time [s]');
subplot(3, 1, 1);
title('0 to 90 deg in 2 s');
legend('cubic', 'quintic', 'trapezoid', 'Location', 'northwest');
