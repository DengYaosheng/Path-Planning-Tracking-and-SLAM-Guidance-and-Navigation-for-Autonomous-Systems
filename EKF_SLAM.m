function [X, P] = EKF_SLAM(X, P, u, z, R, Q, dt, measType)
% EKF_SLAM implements EKF SLAM with range and bearing measurements.
% The state vector X is [x; y; psi; m1_x; m1_y; ...; mN_x; mN_y].
%
% Inputs:
%   X       : current state vector.
%   P       : current covariance.
%   u       : control input [v; psi_dot].
%   z       : measurement matrix, where each column is [r; theta] for each landmark.
%   R       : measurement noise covariance (2x2).
%   Q       : process noise covariance for robot state (3x3).
%   dt      : time step.
%   measType: 'range_bearing' (only type implemented here).
%
% Propagation step (robot motion; landmarks are static).
v = u(1); psi_dot = u(2);
x = X(1); y = X(2); psi = X(3);
x_new = x + dt * v * sin(psi);
y_new = y + dt * v * cos(psi);
psi_new = psi + dt * psi_dot;
X(1:3) = [x_new; y_new; psi_new];
% Jacobian of the robot motion.
F_robot = [1, 0, dt*v*cos(psi);
           0, 1, -dt*v*sin(psi);
           0, 0, 1];
nLand = (length(X)-3)/2;
F = eye(length(X));
F(1:3,1:3) = F_robot;
P = F * P * F' + blkdiag(Q, kron(eye(nLand), zeros(2)));

% Update for each landmark.
for i = 1:nLand
    idx = 3 + 2*i - 1;
    m = X(idx:idx+1); % landmark position
    dx = m(1) - x_new; dy = m(2) - y_new;
    q = dx^2 + dy^2;
    r_pred = sqrt(q);
    theta_pred = angleWrap(atan2(dy, dx) - psi_new);
    z_pred = [r_pred; theta_pred];
    z_actual = z(:,i);
    residual = z_actual - z_pred;
    residual(2) = angleWrap(residual(2));
    
    % Jacobian for robot and landmark.
    H_robot = [-dx/r_pred, -dy/r_pred, 0;
                dy/q,      -dx/q,     -1];
    H_land = [dx/r_pred, dy/r_pred;
             -dy/q,     dx/q];
    H = zeros(2, length(X));
    H(:,1:3) = H_robot;
    H(:, idx:idx+1) = H_land;
    
    S = H * P * H' + R;
    K = P * H' / S;
    X = X + K * residual;
    P = (eye(length(X)) - K * H) * P;
end
end

function ang = angleWrap(ang)
    ang = mod(ang+pi, 2*pi) - pi;
end
