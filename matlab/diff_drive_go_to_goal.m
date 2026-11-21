% Differential-drive robot driving to a goal pose
%
% Kinematics (unicycle model):
%   x'     = v*cos(theta)
%   y'     = v*sin(theta)
%   theta' = w
% Wheel speeds:
%   v = r*(wR + wL)/2,   w = r*(wR - wL)/b
%
% The controller is the polar-coordinate law from Siegwart & Nourbakhsh,
% "Introduction to Autonomous Mobile Robots": it steers the robot to a point
% and also lines it up with the goal heading. Wheel speeds are limited, as
% they would be on a real robot.
%
% Needs MATLAB R2016b or newer (local functions at the end of a script).

r     = 0.035;      % wheel radius [m]
b     = 0.16;       % distance between the wheels [m]
w_max = 15;         % wheel speed limit [rad/s]
dt    = 0.01;       % time step [s]
t_end = 20;

% Gains: k_rho > 0, k_beta < 0, k_alpha > k_rho for a stable approach
k_rho = 0.6;  k_alpha = 2.0;  k_beta = -0.8;

goal  = [1.5; 1.0; deg2rad(90)];          % [x; y; theta]
starts = [0, 0, 0;  0, 1.5, -pi/2;  2.0, 0, pi];

figure;
hold on;
axis equal;
grid on;
for i = 1:size(starts, 1)
    pose = starts(i, :)';
    n = round(t_end / dt);
    path = zeros(3, n);
    for k = 1:n
        [v, w] = go_to_goal(pose, goal, k_rho, k_alpha, k_beta);

        % Convert to wheel speeds, clip them, and convert back
        wR = (2*v + w*b) / (2*r);
        wL = (2*v - w*b) / (2*r);
        scale = max([1, abs(wR) / w_max, abs(wL) / w_max]);
        wR = wR / scale;   wL = wL / scale;      % keeps the turn radius the same
        v = r * (wR + wL) / 2;
        w = r * (wR - wL) / b;

        pose = pose + dt * [v*cos(pose(3)); v*sin(pose(3)); w];
        path(:, k) = pose;

        if norm(pose(1:2) - goal(1:2)) < 0.01 && abs(wrap(pose(3) - goal(3))) < deg2rad(3)
            path = path(:, 1:k);
            break;
        end
    end
    fprintf('Start (%.1f, %.1f, %4.0f deg): reached in %.2f s, position error %.3f m, heading error %.1f deg\n', ...
            starts(i, 1), starts(i, 2), rad2deg(starts(i, 3)), size(path, 2) * dt, ...
            norm(path(1:2, end) - goal(1:2)), rad2deg(wrap(path(3, end) - goal(3))));
    plot(path(1, :), path(2, :), 'LineWidth', 1.5);
    plot(starts(i, 1), starts(i, 2), 'ko', 'MarkerFaceColor', 'k');
end
quiver(goal(1), goal(2), 0.2*cos(goal(3)), 0.2*sin(goal(3)), 0, 'r', 'LineWidth', 2);
hold off;
xlabel('x [m]');
ylabel('y [m]');
title('Go-to-goal from three start poses (goal arrow in red)');


% ---------- Local functions ----------

function [v, w] = go_to_goal(pose, goal, k_rho, k_alpha, k_beta)
    dx = goal(1) - pose(1);
    dy = goal(2) - pose(2);
    rho   = sqrt(dx^2 + dy^2);                     % distance to goal
    alpha = wrap(atan2(dy, dx) - pose(3));         % goal direction vs heading
    beta  = wrap(goal(3) - atan2(dy, dx));         % final heading vs goal direction

    if abs(alpha) > pi/2
        % Goal is behind the robot: drive backwards instead of turning around
        rho = -rho;
        alpha = wrap(alpha + pi);
        beta = wrap(beta + pi);
    end
    v = k_rho * rho;
    w = k_alpha * alpha + k_beta * beta;
end

function a = wrap(a)
    % Wrap an angle to the range [-pi, pi)
    a = mod(a + pi, 2*pi) - pi;
end
