function newParticles = Resample(oldParticles)
global num_particles
weight = zeros(1,num_particles);
for i=1:num_particles
    weight(i) =  oldParticles(i).w;
end

% Normalise the weight
weightSum = sum(weight);
if weightSum > 1e50                                         % the case where weightSum goes infinite
    [~,i] = max(weight);
    weight = zeros(1,num_particles);
    weight(i) = 1;
elseif weightSum > 1e-50                                    % normalise
    weight = weight/weightSum;
else                                                        % the case where weightSum becomes zero
    weight = 1/num_particles*ones(1,num_particles);
end

% Resample
for i = 1:num_particles           
    u = rand ;                                                  % Uniform Random Number Between 0 and 1
    wm = 0 ;                                                    % Accumulated sum of the Likelihood of Each a Priori Estimate
    for j = 1:num_particles
        wm = wm + weight(j);                                    % Compute the Likelihood of Each a Priori Estimate up to j
        if  wm >= u
            newParticles(i) = oldParticles(j);                  % Resample
            break ;
        end
    end
end

% Initialise weightings for the next step
for i=1:num_particles
    newParticles(i).w = 1/num_particles;
end
end
