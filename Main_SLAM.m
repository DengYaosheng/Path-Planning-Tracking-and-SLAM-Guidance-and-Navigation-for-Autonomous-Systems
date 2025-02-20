% Main_SLAM.m
% This script sets up the SLAM scenario and runs:
%   P1: Particle filter SLAM using range and bearing measurements.
%   P2: Particle filter SLAM using bearing-only measurements.
%   P3: (Optional) Data association via nearest neighbor can be activated.
%   P4: EKF SLAM using range and bearing measurements.
%
% It then compares the estimated landmark positions and robot trajectories.

clc; clear; close all;

%% Simulation Parameters
global dt num_landmarks num_particles
dt = 0.1;
timesteps = 100;
time = (1:timesteps)*dt;
global P0
P0 = diag([0.1, 0.1]);

num_landmarks = 20;
% Generate random landmarks (in a 50×50 area)
real_landmarks = -25 + 50*rand(2, num_landmarks);

% Initial robot pose: [x; y; psi]
real_robot = [0; 0; pi/2];

% Control input (constant)
v = 5; psi_dot = 0.1;
u = [v; psi_dot];

% Noise parameters
state_variance = [0.5^2; 0.01^2];
% For measurements:
measurement_variance_rb = diag([1^2, 0.12^2]); % range-bearing
measurement_variance_b = 0.12^2;             % bearing-only

% Process noise covariance for EKF SLAM (robot only)
Q = diag([0.1, 0.1, 0.01]);
% Initial landmark covariance for EKF
P0_land = diag([1, 1]);

% Particle filter parameters
num_particles = 100;

% Initialize particles for PF SLAM
initial_vehicle = real_robot + [randn(2,1); 0];
initial_landmarks = real_landmarks + randn(2, num_landmarks);
particles_rb = Initialisation_particles(initial_vehicle, initial_landmarks);
particles_b  = Initialisation_particles(initial_vehicle, initial_landmarks);

%% Preallocate storage for estimates.
est_robot_PF_rb = zeros(3, timesteps);
est_landmarks_PF_rb = zeros(2, num_landmarks, timesteps);
est_robot_PF_b = zeros(3, timesteps);
est_landmarks_PF_b = zeros(2, num_landmarks, timesteps);

%% EKF SLAM initialization.
% State vector: [robot; landmarks]
X = [real_robot; reshape(initial_landmarks, num_landmarks*2, 1)];
P = blkdiag(Q, kron(eye(num_landmarks), P0_land));

%% Run Simulation
for t = 1:timesteps
    % Propagate true robot state.
    real_robot = Propagation(real_robot, state_variance);
    
    % Generate measurements for each landmark.
    % We assume the sensor has a maximum range (e.g., 15 m).
    z_rb = []; % will be 2 x num_meas_rb (only for landmarks within range)
    z_b = [];  % bearing-only measurements, 1 x num_meas_b
    meas_idx = []; % record landmark indices for which measurement is obtained.
    for l = 1:num_landmarks
        if norm(real_robot(1:2) - real_landmarks(:,l)) < 15
            % Range-bearing measurement with additive noise.
            z_temp = Measurement_SLAM(real_robot, real_landmarks(:,l), 'range_bearing') + ...
                     [sqrt(measurement_variance_rb(1,1))*randn; sqrt(measurement_variance_rb(2,2))*randn];
            z_rb(:,l) = z_temp;  %#ok<AGROW>
            % Bearing-only measurement.
            z_temp_b = Measurement_SLAM(real_robot, real_landmarks(:,l), 'bearing_only') + ...
                       sqrt(measurement_variance_b)*randn;
            z_b(:,l) = z_temp_b; %#ok<AGROW>
            meas_idx(end+1) = l; %#ok<AGROW>
        else
            % If no measurement, leave as empty (or zero).
            z_rb(:,l) = []; 
            z_b(:,l) = [];
        end
    end
    
    % Update PF SLAM (range-bearing)
    particles_rb = SLAM_PF_mod(particles_rb, z_rb, 'range_bearing', false);
    % Estimate robot pose and landmarks from particles (averaging)
    robot_est_rb = zeros(3,1);
    landmarks_est_rb = zeros(2, num_landmarks);
    for p = 1:num_particles
        robot_est_rb = robot_est_rb + particles_rb(p).position;
        for l = 1:num_landmarks
            landmarks_est_rb(:,l) = landmarks_est_rb(:,l) + particles_rb(p).landmarks(l).pos;
        end
    end
    robot_est_rb = robot_est_rb / num_particles;
    landmarks_est_rb = landmarks_est_rb / num_particles;
    est_robot_PF_rb(:,t) = robot_est_rb;
    est_landmarks_PF_rb(:,:,t) = landmarks_est_rb;
    
    % Update PF SLAM (bearing-only)
    particles_b = SLAM_PF_mod(particles_b, z_b, 'bearing_only', false);
    robot_est_b = zeros(3,1);
    landmarks_est_b = zeros(2, num_landmarks);
    for p = 1:num_particles
        robot_est_b = robot_est_b + particles_b(p).position;
        for l = 1:num_landmarks
            landmarks_est_b(:,l) = landmarks_est_b(:,l) + particles_b(p).landmarks(l).pos;
        end
    end
    robot_est_b = robot_est_b / num_particles;
    landmarks_est_b = landmarks_est_b / num_particles;
    est_robot_PF_b(:,t) = robot_est_b;
    est_landmarks_PF_b(:,:,t) = landmarks_est_b;
    
    % EKF SLAM update (range-bearing).
    [X, P] = EKF_SLAM(X, P, u, z_rb, measurement_variance_rb, Q, dt, 'range_bearing');
    ekf_robot(:,t) = X(1:3);
    ekf_landmarks(:,:,t) = reshape(X(4:end), [2, num_landmarks]);
end

%% Plot final landmark estimates.
figure;
subplot(121);
plot(real_landmarks(1,:), real_landmarks(2,:), 'b*', 'MarkerSize', 10); hold on;
plot(ekf_landmarks(1,:,end), ekf_landmarks(2,:,end), 'ro', 'MarkerSize', 8);
legend('True Landmarks', 'EKF SLAM');
title('EKF SLAM Landmark Estimates');
xlabel('x [m]'); ylabel('y [m]');
axis equal;

subplot(122);
plot(real_landmarks(1,:), real_landmarks(2,:), 'b*', 'MarkerSize', 10); hold on;
plot(est_landmarks_PF_rb(1,:,end), est_landmarks_PF_rb(2,:,end), 'gs', 'MarkerSize', 8);
legend('True Landmarks', 'PF SLAM (Range-Bearing)');
title('PF SLAM Landmark Estimates (Range-Bearing)');
xlabel('x [m]'); ylabel('y [m]');
axis equal;

figure;
plot(ekf_robot(1,:), ekf_robot(2,:), 'r-', 'LineWidth',2); hold on;
plot(est_robot_PF_rb(1,:), est_robot_PF_rb(2,:), 'b--', 'LineWidth',2);
plot(est_robot_PF_b(1,:), est_robot_PF_b(2,:), 'g:', 'LineWidth',2);
legend('EKF SLAM Robot', 'PF SLAM (Range-Bearing)', 'PF SLAM (Bearing-Only)');
title('Robot Trajectory Comparison');
xlabel('x [m]'); ylabel('y [m]');
axis equal;
