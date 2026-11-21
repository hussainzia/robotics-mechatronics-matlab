% Two-link planar arm: forward kinematics, inverse kinematics and Jacobian
%
% Joint angles are measured from the x-axis (theta1) and from link 1 (theta2).
% The script checks that IK followed by FK gets back to the same point, then
% makes the arm trace a circle and plots it.
%
% Needs MATLAB R2016b or newer (local functions at the end of a script).

L1 = 1.0;   % link 1 length [m]
L2 = 0.7;   % link 2 length [m]

% ---- 1. Forward kinematics for one pose ----
q = [deg2rad(30); deg2rad(45)];
p = fk(q, L1, L2);
fprintf('FK at (30, 45) deg: x = %.4f m, y = %.4f m\n', p(1), p(2));

% Same answer from the DH table, using 4x4 homogeneous transforms
%        a    alpha  d   theta
dh = [  L1    0     0   q(1);
        L2    0     0   q(2) ];
T = eye(4);
for i = 1:size(dh, 1)
    T = T * dh_transform(dh(i, 1), dh(i, 2), dh(i, 3), dh(i, 4));
end
fprintf('FK from DH table:   x = %.4f m, y = %.4f m\n', T(1, 4), T(2, 4));

% ---- 2. Inverse kinematics round trip ----
target = [1.2; 0.6];
[q_down, q_up, ok] = ik(target, L1, L2);
if ok
    fprintf('IK elbow-down: (%.2f, %.2f) deg, FK error %.2e m\n', rad2deg(q_down), ...
            norm(fk(q_down, L1, L2) - target));
    fprintf('IK elbow-up:   (%.2f, %.2f) deg, FK error %.2e m\n', rad2deg(q_up), ...
            norm(fk(q_up, L1, L2) - target));
end

% ---- 3. Jacobian: joint speeds -> tool speed ----
Jq = arm_jacobian(q, L1, L2);
qdot = [0.5; -0.2];                    % [rad/s]
v = Jq * qdot;
fprintf('Tool velocity at (30, 45) deg: vx = %.3f, vy = %.3f m/s\n', v(1), v(2));
fprintf('det(J) = %.3f (zero means a singular pose, e.g. arm fully stretched)\n', det(Jq));

% ---- 4. Trace a circle with IK ----
center = [1.0; 0.5];  radius = 0.3;
angles = linspace(0, 2*pi, 120);
q_path = zeros(2, length(angles));
for k = 1:length(angles)
    pt = center + radius * [cos(angles(k)); sin(angles(k))];
    [q_path(:, k), ~, ok] = ik(pt, L1, L2);
    if ~ok
        error('Point %d is out of reach', k);
    end
end

figure;
hold on;
axis equal;
grid on;
tool = zeros(2, length(angles));
for k = 1:length(angles)
    tool(:, k) = fk(q_path(:, k), L1, L2);
end
plot(tool(1, :), tool(2, :), 'b-', 'LineWidth', 1.5);
for k = 1:15:length(angles)            % draw the arm at a few poses
    elbow = L1 * [cos(q_path(1, k)); sin(q_path(1, k))];
    plot([0 elbow(1) tool(1, k)], [0 elbow(2) tool(2, k)], 'k-o', 'LineWidth', 1);
end
hold off;
xlabel('x [m]');
ylabel('y [m]');
title('Two-link arm tracing a circle (elbow-down IK)');

figure;
plot(rad2deg(angles), rad2deg(q_path), 'LineWidth', 1.5);
grid on;
xlabel('Position around the circle [deg]');
ylabel('Joint angle [deg]');
legend('\theta_1', '\theta_2');
title('Joint angles needed for the circle');


% ---------- Local functions ----------

function p = fk(q, L1, L2)
    % Tool position [x; y] for joint angles q = [theta1; theta2]
    p = [L1*cos(q(1)) + L2*cos(q(1) + q(2));
         L1*sin(q(1)) + L2*sin(q(1) + q(2))];
end

function [q_down, q_up, ok] = ik(p, L1, L2)
    % Both IK solutions for a point p. ok is false if p is out of reach.
    x = p(1);  y = p(2);
    c2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    ok = abs(c2) <= 1;
    if ~ok
        q_down = [NaN; NaN];  q_up = [NaN; NaN];
        return;
    end
    s2 = sqrt(1 - c2^2);
    % For a target to the right of the base, a positive theta2 puts the
    % elbow below the line from base to target (elbow-down).
    q2_down = atan2( s2, c2);
    q2_up   = atan2(-s2, c2);
    q_down = [atan2(y, x) - atan2(L2*sin(q2_down), L1 + L2*cos(q2_down)); q2_down];
    q_up   = [atan2(y, x) - atan2(L2*sin(q2_up),   L1 + L2*cos(q2_up));   q2_up];
end

function J = arm_jacobian(q, L1, L2)
    % 2x2 Jacobian: [vx; vy] = J * [dtheta1; dtheta2]
    s1 = sin(q(1));  c1 = cos(q(1));
    s12 = sin(q(1) + q(2));  c12 = cos(q(1) + q(2));
    J = [-L1*s1 - L2*s12,  -L2*s12;
          L1*c1 + L2*c12,   L2*c12];
end

function T = dh_transform(a, alpha, d, theta)
    % Standard Denavit-Hartenberg transform from frame i-1 to frame i
    ct = cos(theta);  st = sin(theta);
    ca = cos(alpha);  sa = sin(alpha);
    T = [ct, -st*ca,  st*sa, a*ct;
         st,  ct*ca, -ct*sa, a*st;
          0,     sa,     ca,    d;
          0,      0,      0,    1];
end
