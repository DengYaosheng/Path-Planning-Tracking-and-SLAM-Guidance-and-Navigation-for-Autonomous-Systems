function particles_SF = Fusion_state(particles_vehicle1, particles_vehicle2, particles_SF, index_fov)
global num_landmarks num_particles
for l = 1:num_landmarks     
    for p = 1:num_particles                   
        if index_fov(3,l) % seen by both vehicles
            a = particles_vehicle1(p).landmarks(l).pos;
            b = particles_vehicle2(p).landmarks(l).pos;
            A = particles_vehicle1(p).landmarks(l).P;
            B = particles_vehicle2(p).landmarks(l).P;
            
            particles_SF(p).landmarks(l).pos = a+A/(A+B)*(b-a);
            particles_SF(p).landmarks(l).P   = A-A/(A+B)*A';
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