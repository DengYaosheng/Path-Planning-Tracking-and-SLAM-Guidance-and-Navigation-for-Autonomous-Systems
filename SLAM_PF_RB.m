function particles = SLAM_PF_RB(particles, z, index_fov)
% SLAM_PF_RB: Particle filter SLAM using range and bearing measurements.
%
% Inputs:
%   particles  - structure array of particles, where each particle has:
%                   .position : vehicle state [x; y; psi]
%                   .landmarks(l).pos : landmark position [lx; ly]
%                   .landmarks(l).P   : landmark covariance (2x2)
%                   .w        : particle weight
%   z          - measurement matrix of size 2 x num_landmarks, where column l is:
%                   [r; theta] measurement for landmark l.
%   index_fov  - binary vector (1 x num_landmarks) indicating whether each landmark 
%                is within the field of view.
%
% Global variables:
%   num_landmarks - number of landmarks
%   num_particles - number of particles
%   Q             - process noise covariance for vehicle propagation (used in Propagation.m)
%   R_RB          - measurement noise covariance for range-bearing [2x2]
%
% The function performs the predict-update-resample cycle for PF SLAM.
    
global num_landmarks num_particles Q R_RB

%% Prediction: Propagate vehicle state for each particle
for p = 1:num_particles
    particles(p).position = Propagation(particles(p).position, diag(Q));
end

%% Update: For each landmark that is observed, update each particle's landmark estimate.
doResample = false;
for l = 1:num_landmarks
    if index_fov(l)  % landmark l is within FOV
        doResample = true;
        for p = 1:num_particles
            % Predicted measurement using range-bearing model:
            z_p = Measurement_RB(particles(p).position, particles(p).landmarks(l).pos);
            % Compute measurement residual (innovation):
            residual = z(:, l) - z_p;
            % Wrap bearing error to [-pi, pi]
            residual(2) = mod(residual(2) + pi, 2*pi) - pi;
            
            % Compute Jacobian H of the measurement with respect to landmark position.
            % Let dx = lx - x, dy = ly - y, r = sqrt(dx^2 + dy^2)
            x_v = particles(p).position(1);
            y_v = particles(p).position(2);
            lx = particles(p).landmarks(l).pos(1);
            ly = particles(p).landmarks(l).pos(2);
            dx_val = lx - x_v;
            dy_val = ly - y_v;
            r_val = sqrt(dx_val^2 + dy_val^2);
            if r_val < 1e-6
                r_val = 1e-6;  % avoid division by zero
            end
            H = [dx_val/r_val, dy_val/r_val;
                 -dy_val/(r_val^2), dx_val/(r_val^2)];
             
            % Get the current landmark covariance:
            P_l = particles(p).landmarks(l).P;
            % Innovation covariance:
            S = H * P_l * H' + R_RB;
            % Kalman gain:
            K = P_l * H' / S;
            
            % EKF update for the landmark estimate:
            particles(p).landmarks(l).pos = particles(p).landmarks(l).pos + K * residual;
            particles(p).landmarks(l).P = (eye(2) - K * H) * P_l;
            
            % Update the particle's weight based on measurement likelihood.
            % (Here, we use a Gaussian likelihood function.)
            likelihood = 1 / sqrt(det(2*pi*S)) * exp(-0.5 * (residual' / S * residual));
            particles(p).w = particles(p).w * likelihood;
        end
    end
end

%% Resample: if any landmark was updated, resample the particles based on their weights.
if doResample
    particles = Resample(particles);
end
