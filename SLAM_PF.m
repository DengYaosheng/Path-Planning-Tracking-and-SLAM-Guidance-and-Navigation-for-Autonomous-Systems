function [particles]= SLAM_PF(particles,z,index_fov)
global num_landmarks num_particles Q R 
%% Predict
for p = 1:num_particles
    particles(p).position = Propagation(particles(p).position,diag(Q));
end

%% Update
doResample = false;
for l = 1:num_landmarks
    if index_fov(l) % If within field of view
        doResample = true;        
        for p = 1:num_particles           
            z_p = Measurement(particles(p).position,particles(p).landmarks(l).pos);
            residual = z(:,l) - z_p;

            % Calculate the Kalman gain
            H = eye(2);
            P = particles(p).landmarks(l).P;
            S = H*P*H' + R;
            K = P*H'/S;

            % Update EKF
            particles(p).landmarks(l).pos = particles(p).landmarks(l).pos + K*residual;
            particles(p).landmarks(l).P = (eye(2)-K*H)*P;

            % Update particle filter
            particles(p).w = particles(p).w*1/sqrt(norm(2*pi*S))*exp(-1/2*residual'/S*residual);
        end
    end
end

% Resample all particles based on their weights
if doResample
    particles = Resample(particles);
end



