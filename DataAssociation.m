function landmark_idx = DataAssociation(z, pos_vehicle, landmarks, measType, R)
% DataAssociation finds the landmark index that best matches the measurement z
% (using a nearest-neighbor Mahalanobis distance).
%
% Inputs:
%   z          : the actual measurement (2×1 for range_bearing, scalar for bearing_only)
%   pos_vehicle: vehicle pose [x; y; psi]
%   landmarks  : matrix of landmark positions (2 x N)
%   measType   : 'range_bearing' or 'bearing_only'
%   R          : measurement noise covariance (2×2 for range_bearing,
%                scalar for bearing_only)
%
% Output:
%   landmark_idx : index of the associated landmark.
n_landmarks = size(landmarks, 2);
min_dist = inf;
landmark_idx = -1;
for i = 1:n_landmarks
    pred = Measurement_SLAM(pos_vehicle, landmarks(:,i), measType);
    if strcmp(measType, 'range_bearing')
        residual = z - pred;
        residual(2) = angleWrap(residual(2));
        dist = residual' / R * residual;
    else
        residual = angleWrap(z - pred);
        dist = (residual^2) / R;
    end
    if dist < min_dist
        min_dist = dist;
        landmark_idx = i;
    end
end
end
