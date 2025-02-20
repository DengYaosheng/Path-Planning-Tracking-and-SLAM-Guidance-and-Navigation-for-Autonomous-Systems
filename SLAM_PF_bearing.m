function particles = SLAM_PF_bearing(particles, z, index_fov)
% SLAM_PF_bearing：使用仅有方位角量测的粒子滤波 SLAM 更新步骤
%
% 输入：
%   particles  - 粒子结构体数组，每个粒子包含：
%                   .position：车辆状态 [x; y; psi]
%                   .landmarks(l).pos：第 l 个路标位置 [lx; ly]
%                   .landmarks(l).P  ：第 l 个路标估计协方差 (2x2)
%                   .w        ：粒子权重
%   z          - 1×num_landmarks 的量测向量，其中 z(l) 为对第 l 个路标的方位角量测（单位：rad）
%   index_fov  - 1×num_landmarks 的二值数组，指示对应路标是否在视野内
%
% 全局变量：
%   num_landmarks - 路标总数
%   num_particles - 粒子数
%   Q             - Propagation 时车辆状态噪声协方差（用于 Propagation.m）
%   R_bearing     - 方位角量测噪声方差（标量），例如设为 (0.12)^2
%
% 更新步骤：
%   1. 对每个粒子进行预测（调用 Propagation.m）
%   2. 对于视野内的每个路标，计算预测方位角（使用 Measurement_SLAM.m 中 'bearing_only' 分支），
%      利用 EKF 更新公式更新路标状态及协方差，同时更新粒子权重
%   3. 如有更新，则调用 Resample.m 重采样粒子

global num_landmarks num_particles Q R_bearing

%% 预测：对每个粒子，调用 Propagation.m 更新车辆状态
for p = 1:num_particles
    particles(p).position = Propagation(particles(p).position, diag(Q));
end

%% 更新
doResample = false;
for l = 1:num_landmarks
    if index_fov(l)  % 如果路标 l 在视野内
        doResample = true;
        for p = 1:num_particles
            % 计算预测的方位角量测，调用 Measurement_SLAM，测量类型固定为 'bearing_only'
            z_p = Measurement_SLAM(particles(p).position, particles(p).landmarks(l).pos, 'bearing_only');
            % 真实量测 z(l) 为标量
            residual = z(l) - z_p;
            % 将角度误差包装到 [-pi, pi]
            residual = mod(residual + pi, 2*pi) - pi;
            
            % 计算测量模型的雅可比 H（1×2），
            % 设 dx = lx - x, dy = ly - y, 则：
            %   H = [-dy/(r^2)  ,  dx/(r^2)]
            x_v = particles(p).position(1);
            y_v = particles(p).position(2);
            lx  = particles(p).landmarks(l).pos(1);
            ly  = particles(p).landmarks(l).pos(2);
            dx_val = lx - x_v;
            dy_val = ly - y_v;
            r_sq = dx_val^2 + dy_val^2;
            % 防止 r_sq 过小
            if r_sq < 1e-6
                r_sq = 1e-6;
            end
            H = [-dy_val / r_sq, dx_val / r_sq];  % 1×2
            
            % 当前路标的协方差
            P_l = particles(p).landmarks(l).P;
            % 创新协方差
            S = H * P_l * H' + R_bearing;
            % Kalman 增益
            K = P_l * H' / S;
            
            % EKF 更新路标状态
            particles(p).landmarks(l).pos = particles(p).landmarks(l).pos + K * residual;
            particles(p).landmarks(l).P = (eye(2) - K * H) * P_l;
            
            % 利用高斯似然更新粒子权重（注意 residual 为标量）
            likelihood = 1 / sqrt(2*pi*S) * exp(-0.5 * (residual^2) / S);
            particles(p).w = particles(p).w * likelihood;
        end
    end
end

%% 重采样：若有更新，则基于粒子权重重采样
if doResample
    particles = Resample(particles);
end
