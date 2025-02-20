function particles = SLAM_PF_mod(particles, z, measType, useDataAssociation)
% SLAM_PF_mod performs one update of the particle filter SLAM.
%
% Inputs:
%   particles         : array of particle structures, each with fields:
%                         .position, .landmarks (each with .pos and .P), .w (weight)
%   z                 : measurement matrix. For each landmark l, z(:,l) is provided if a measurement is available.
%   measType          : 'range_bearing' or 'bearing_only'
%   useDataAssociation: if true, use NN data association (here shown as a placeholder)
%
% Global:
%   num_landmarks, num_particles, R
%
% Propagate each particle.
global num_landmarks num_particles R
for p = 1:num_particles
    particles(p).position = Propagation(particles(p).position, diag(R)); 
end

% Update each landmark estimate for each particle if measurement is available.
for l = 1:num_landmarks
    % If measurement available for landmark l (non-empty column)
    if ~isempty(z(:,l))
        for p = 1:num_particles
            % If data association is enabled, one would select the landmark index here.
            % For simplicity, we assume known association if useDataAssociation is false.
            % (If true, you could call: idx = DataAssociation(z(:,l), particles(p).position, ...))
            idx = l;  % assumed correct association

            % Predicted measurement from particle p for landmark idx.
            z_pred = Measurement_SLAM(particles(p).position, particles(p).landmarks(idx).pos, measType);
            if strcmp(measType, 'range_bearing')
                residual = z(:,l) - z_pred;
                residual(2) = angleWrap(residual(2));
            else
                residual = angleWrap(z(:,l) - z_pred);
            end
            
            % Here we assume a linearized measurement with H = identity (a simplification)
            if strcmp(measType, 'range_bearing')
                H = eye(2);
            else
                H = 1;
            end
            
            P_land = particles(p).landmarks(idx).P;
            S = H * P_land * H' + R;
            K = P_land * H' / S;
            % Update landmark estimate (EKF-style update)
            particles(p).landmarks(idx).pos = particles(p).landmarks(idx).pos + K * residual;
            particles(p).landmarks(idx).P = (eye(2) - K * H) * P_land;
            
            % Update weight using likelihood
            if strcmp(measType, 'range_bearing')
                particles(p).w = particles(p).w * 1/sqrt(det(2*pi*S)) * exp(-0.5 * residual' / S * residual);
            else
                particles(p).w = particles(p).w * 1/sqrt(2*pi*S) * exp(-0.5 * (residual^2)/S);
            end
        end
    end
end

% Normalize weights.
w_total = sum([particles.w]);
if w_total == 0
    for p = 1:num_particles
        particles(p).w = 1/num_particles;
    end
else
    for p = 1:num_particles
        particles(p).w = particles(p).w / w_total;
    end
end

% Resample particles.
particles = Resample(particles);
end
