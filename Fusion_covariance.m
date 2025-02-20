function particles_SF = Fusion_covariance(particles_vehicle1, particles_vehicle2, particles_SF, index_fov)
global num_landmarks num_particles 
for l = 1:num_landmarks
    for p = 1:num_particles
        if index_fov(3,l) % seen by both vehicles    
            a = particles_vehicle1(p).landmarks(l).pos;
            b = particles_vehicle2(p).landmarks(l).pos;
            inv_A = inv(particles_vehicle1(p).landmarks(l).P);
            inv_B = inv(particles_vehicle2(p).landmarks(l).P);

            covariance_det = @(omega) 1/det(omega*inv_A+(1-omega)*inv_B);
            omega = fminbnd(covariance_det,0,1);
%             omega = 0.5;
            particles_SF(p).landmarks(l).P = inv(omega*inv_A+(1-omega)*inv_B);
            particles_SF(p).landmarks(l).pos = particles_SF(p).landmarks(l).P*(omega*inv_A*a+(1-omega)*inv_B*b);     
            continue;
        end
        if index_fov(1,l) % seen by vehicle1 only
            particles_SF(p).landmarks(l).pos = particles_vehicle1(p).landmarks(l).pos;
            particles_SF(p).landmarks(l).P   = particles_vehicle1(p).landmarks(l).P;
        end
        if index_fov(2,l) % seen by vehicle2 only
            particles_SF(p).landmarks(l).pos = particles_vehicle2(p).landmarks(l).pos;
            particles_SF(p).landmarks(l).P   = particles_vehicle2(p).landmarks(l).P;
        end
    end
end

