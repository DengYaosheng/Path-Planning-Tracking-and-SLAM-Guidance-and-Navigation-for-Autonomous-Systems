%% main_SLAM_P3.m
% 本脚本实现 Task2 P3：在粒子滤波 SLAM 中对相对 x-y 量测进行数据关联。
% 采用最近邻关联（基于马氏距离）更新粒子中对应路标的估计。
% 本脚本基于已提供的 13 份代码（例如 Propagation.m、Resample.m、Initialisation_particles.m、Measurement.m 等）
% 请确保这些文件均在 MATLAB 路径中。



%% 设置全局变量
global num_landmarks num_particles Q R
num_landmarks = 5;
num_particles = 50;
% 车辆状态传播噪声（状态 [x;y;psi]）
Q = diag([0.5^2, 0.5^2, 0.01^2]);
% 量测噪声协方差（相对坐标测量，单位：m^2）
R = diag([1, 1]);  % 可根据需要调整

%% 定义场景：真值
% 真路标位置（2 x num_landmarks）
true_landmarks = [ -10,   0,  10,   5,  -5;
                   10,    5,  -5, -10,   0];
% 真车辆状态 [x; y; psi]，设车辆位于原点，航向 0 rad（朝 x 轴正方向）
true_vehicle = [0; 0; 0];

%% 模拟测量
% 对每个路标，利用 Measurement.m 生成无噪声相对坐标量测（z = landmark - vehicle(1:2)），再加上噪声
% 得到的 z 为 2 x num_landmarks，但我们将打乱顺序来模拟未知关联情况
z_all = zeros(2, num_landmarks);
for l = 1:num_landmarks
    % 计算无噪声测量（基于真值）
    z0 = Measurement(true_vehicle, true_landmarks(:,l));
    % 加噪声（噪声协方差 R）
    noise = [sqrt(R(1,1))*randn; sqrt(R(2,2))*randn];
    z_all(:,l) = z0 + noise;
end
% 打乱测量顺序：随机排列列索引
perm_idx = randperm(num_landmarks);
z_meas = z_all(:, perm_idx);
% 同时，设置视野标记：假设所有测量都有效
index_fov_meas = ones(1, size(z_meas,2));  % 1 x m，其中 m=num_landmarks

%% 初始化粒子（调用 Initialisation_particles.m）
% 假设初始车辆状态接近真值，但初始路标估计存在较大误差
initial_vehicle = true_vehicle + [0.5*randn; 0.5*randn; 0.05*randn];
initial_landmarks = true_landmarks + 2*randn(2, num_landmarks);
particles = Initialisation_particles(initial_vehicle, initial_landmarks);

%% 数据关联更新
% 对于每个粒子，遍历所有接收到的测量（顺序已打乱），进行最近邻数据关联
% 关联时使用的度量：马氏距离： d^2 = residual' * inv(P + R) * residual
assoc_threshold = 9;  % 设定关联阈值（例如：9 对应 3-sigma 检验）

for p = 1:num_particles
    % 当前粒子的车辆状态
    veh = particles(p).position;
    % 对于每个测量（共 m 个）
    for j = 1:size(z_meas,2)
        z_current = z_meas(:,j);  % 2x1 测量
        % 对当前粒子中每个路标，计算预测测量及马氏距离
        min_dist = inf;
        assoc_l = -1;
        for l = 1:num_landmarks
            % 预测测量： landmark 估计 - 车辆位置（前两分量）
            z_pred = particles(p).landmarks(l).pos - veh(1:2);
            % 残差
            residual = z_current - z_pred;
            % 计算马氏距离：d^2 = residual' * inv(P + R) * residual
            P_l = particles(p).landmarks(l).P;
            S = P_l + R;
            d2 = residual' / S * residual;  % 注意 S 为 2×2
            if d2 < min_dist
                min_dist = d2;
                assoc_l = l;
            end
        end
        % 如果最小马氏距离低于阈值，则认为测量与 landmark assoc_l 对应，进行 EKF 更新
        if min_dist < assoc_threshold && assoc_l > 0
            % 对于该粒子和 landmark assoc_l，使用线性模型：测量函数为 h = landmark - veh(1:2)，雅可比 H = eye(2)
            H = eye(2);
            P_l = particles(p).landmarks(assoc_l).P;
            S = H * P_l * H' + R;
            K = P_l * H' / S;
            % 残差（重新计算，用于更新）
            z_pred = particles(p).landmarks(assoc_l).pos - veh(1:2);
            residual = z_current - z_pred;
            % 更新 landmark 状态和协方差
            particles(p).landmarks(assoc_l).pos = particles(p).landmarks(assoc_l).pos + K * residual;
            particles(p).landmarks(assoc_l).P = (eye(2) - K * H) * P_l;
            % 更新粒子权重（乘以似然值）
            likelihood = 1 / sqrt(det(2*pi*S)) * exp(-0.5 * residual' / S * residual);
            particles(p).w = particles(p).w * likelihood;
        end
    end
end

%% 重采样（调用 Resample.m）
particles = Resample(particles);

%% 对所有粒子中各路标估计取均值作为最终估计
est_landmarks = zeros(2, num_landmarks);
for l = 1:num_landmarks
    sum_est = zeros(2,1);
    for p = 1:num_particles
        sum_est = sum_est + particles(p).landmarks(l).pos;
    end
    est_landmarks(:,l) = sum_est / num_particles;
end

%% 显示结果
disp('真值 Landmark:');
disp(true_landmarks);
disp('数据关联后，粒子滤波 SLAM 估计（取均值）:');
disp(est_landmarks);

%% 绘图对比
figure; hold on; grid on;
plot(true_landmarks(1,:), true_landmarks(2,:), 'bo', 'MarkerSize',10, 'LineWidth',2);
plot(est_landmarks(1,:), est_landmarks(2,:), 'rx', 'MarkerSize',10, 'LineWidth',2);
legend('真值 Landmark', '估计 Landmark');
xlabel('x (m)'); ylabel('y (m)');
title('带数据关联的粒子滤波 SLAM 结果');
