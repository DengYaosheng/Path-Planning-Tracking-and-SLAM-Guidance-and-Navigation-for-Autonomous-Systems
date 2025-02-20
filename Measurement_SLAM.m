function meas = Measurement_SLAM(pos_vehicle, pos_landmark, measType)
% Measurement_SLAM computes the sensor measurement from a vehicle to a landmark.
% Input:
%   pos_vehicle  : [x; y; psi] (only x,y used)
%   pos_landmark : [x; y]
%   measType     : 'range_bearing' returns [r; theta]
%                  'bearing_only' returns theta (scalar)
%
% The measurement noise is added externally.
dx = pos_landmark(1) - pos_vehicle(1);
dy = pos_landmark(2) - pos_vehicle(2);
if strcmp(measType, 'range_bearing')
    r = sqrt(dx^2 + dy^2);
    theta = atan2(dy, dx);
    meas = [r; theta];
elseif strcmp(measType, 'bearing_only')
    theta = atan2(dy, dx);
    meas = theta;
else
    error('Unknown measurement type.');
end
end

function ang = angleWrap(ang)
    % Wrap angle to [-pi, pi]
    ang = mod(ang+pi, 2*pi) - pi;
end
