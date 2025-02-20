function meas = Measurement(pos_vehicle, pos_landmarks)
% Measurement equation
meas = pos_landmarks - pos_vehicle(1:2);
end
