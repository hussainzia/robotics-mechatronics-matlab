% Mass-spring-damper: step response for three damping values
%
%   m*x'' + c*x' + k*x = F
%
% State vector: s = [x; v]

m = 1.0;          % mass [kg]
k = 20;           % spring stiffness [N/m]
F = 10;           % step force [N]
t_end = 6;        % simulation time [s]

zetas = [0.1, 0.5, 1.0];          % damping ratios to compare
wn = sqrt(k / m);                 % natural frequency [rad/s]
fprintf('Natural frequency: %.2f rad/s (%.2f Hz)\n', wn, wn / (2*pi));
fprintf('Static deflection F/k: %.3f m\n', F / k);

figure;
hold on;
for i = 1:length(zetas)
    c = 2 * zetas(i) * sqrt(k * m);          % damping coefficient [N*s/m]
    rhs = @(t, s) [s(2); (F - c*s(2) - k*s(1)) / m];
    [t, s] = ode45(rhs, [0 t_end], [0; 0]);
    plot(t, s(:, 1), 'LineWidth', 1.5);

    overshoot = (max(s(:, 1)) - F/k) / (F/k) * 100;
    fprintf('zeta = %.1f  ->  c = %.2f N*s/m, overshoot = %.1f %%\n', ...
            zetas(i), c, max(overshoot, 0));
end
plot([0 t_end], [F/k F/k], 'k--');
hold off;
grid on;
xlabel('Time [s]');
ylabel('Position x [m]');
title('Mass-spring-damper step response');
legend('\zeta = 0.1', '\zeta = 0.5', '\zeta = 1.0', 'F/k', 'Location', 'southeast');
