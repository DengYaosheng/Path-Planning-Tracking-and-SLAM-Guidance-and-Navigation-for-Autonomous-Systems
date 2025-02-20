global dt num_landmarks vel_cmd
global Q R Rm P0 num_particles

%% Simulation parameters
dt = 0.1;
timesteps = 100;
time = (1:timesteps)*dt;

% landmarks
num_landmarks = 50;
real_landmarks = -25+50*rand(2,num_landmarks);

% vehicles
real_vehicle1 = zeros(3,timesteps);
real_vehicle2 = zeros(3,timesteps);
real_vehicle1(:,1) = [-10; 20; 300*pi/180];     % x (m) ; y (m) ; rotation (rad)
real_vehicle2(:,1) = [-20;  5; 310*pi/180];     % x (m) ; y (m) ; rotation (rad)
vel_cmd        = [5; 0.1];                  % Speed (m/s) ; Angular Rate (rad/s)

% noise settings
state_variance = [0.5^2; 0.01^2];        % Speed (m^2/s^2) ; Angular Rate (rad^2/s^2)
distance_variance = [1^2; 1^2];             % Distance x (m^2) ; Distance y (m^2)
measurement_variance = [1^2; 1^2];       % Distance x (m^2) ; Distance y (m^2)

% The field of range of a sensor (can sense a landmark)
max_read_distance = 15;

%% Create trajectory/measurement
% The initial measurements
meas_landmark1 = zeros(length(measurement_variance),num_landmarks,timesteps);
meas_landmark2 = zeros(length(measurement_variance),num_landmarks,timesteps);
meas_landmark1(:,:,1) = Measurement(real_vehicle1(:,1),real_landmarks+sqrt(diag(distance_variance))*randn(2,1));
meas_landmark2(:,:,1) = Measurement(real_vehicle2(:,1),real_landmarks+sqrt(diag(distance_variance))*randn(2,1));

index_seen = zeros(3,num_landmarks,timesteps); % Index for landmark has been discovered at least once (1: vehicle1, 2: vehicle2, 3: Fusion)
index_fov = zeros(3,num_landmarks,timesteps);  % Index for landmark is close enough (1: vehicle1, 2: vehicle2, 3: Fusion)
for t = 2:timesteps
    % Move the vehicle
    real_vehicle1(:,t) = Propagation(real_vehicle1(:,t-1),state_variance);
    real_vehicle2(:,t) = Propagation(real_vehicle2(:,t-1),state_variance);
    
    % Get the measurements if the landmark is close enough    
    for l = 1:num_landmarks
        if sqrt(sum((real_vehicle1(1:2,t)-real_landmarks(:,l)).^2))<max_read_distance
            meas_landmark1(:,l,t) = Measurement(real_vehicle1(:,t),real_landmarks(:,l)+sqrt(diag(distance_variance))*randn(2,1));
            index_seen(1,l,t:end) = 1;
            index_fov(1,l,t) = 1;
        end
        if sqrt(sum((real_vehicle2(1:2,t)-real_landmarks(:,l)).^2))<max_read_distance
            meas_landmark2(:,l,t) = Measurement(real_vehicle2(:,t),real_landmarks(:,l)+sqrt(diag(distance_variance))*randn(2,1));
            index_seen(2,l,t:end) = 1;
            index_fov(2,l,t) = 1;
        end
    end
end
index_seen(3,:,:) = index_seen(1,:,:).*index_seen(2,:,:);
index_fov(3,:,:) = index_fov(1,:,:).*index_fov(2,:,:);

%% EKF parameters
Q = diag(state_variance);                                   % State covariance
R = diag(measurement_variance);                             % Measurement covariance
Rm = diag([measurement_variance;measurement_variance]);     % Measurement covariance for measurement fusion
P0 = diag([0.1 0.1]);

%% Particle filter parameters
num_particles = 100;

%% Initial estimates
initial_landmarks = real_landmarks + [1;1].*randn(2,num_landmarks); 
initial_vehicle1 = real_vehicle1(:,1) + [1;1;0].*randn(3,1); 
initial_vehicle2 = real_vehicle2(:,1) + [1;1;0].*randn(3,1); 
             



