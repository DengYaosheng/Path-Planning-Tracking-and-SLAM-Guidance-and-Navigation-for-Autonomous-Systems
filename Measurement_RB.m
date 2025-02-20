% File: Measurement_RB.m
function meas = Measurement_RB(pos_vehicle, pos_landmark)
    % Compute range and bearing measurements
    dx = pos_landmark(1) - pos_vehicle(1);
    dy = pos_landmark(2) - pos_vehicle(2);
    r = sqrt(dx^2 + dy^2);
    theta = atan2(dy, dx); % bearing measurement (in rad)
    meas = [r; theta];
end
