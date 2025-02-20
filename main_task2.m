%% main_SLAM_P1.m
% 本脚本用于测试 Task 2 P1，即基于 range‐and‐bearing 测量的粒子滤波 SLAM
% 它调用了你提供的 SLAM_PF.m 与 Measurement_SLAM.m（不修改），
% 通过定义一个 Measurement 包装函数使得 SLAM_PF 内调用的 Measurement 返回 [r; theta]
% （测量类型固定为 'range_bearing'）。


%% 设置全局变量
global num_landmarks num_particles Q R 
% 注意：SLAM_PF.m 使用全局变量 R 作为测量噪声协方差，本例中我们令
% R = diag([1, (0.12)^2]);  表示量测噪声：距离方差为1，角度方差为 (0.12)^2
num_landmarks = 5;
num_particles = 50;
Q = diag([0.5^2, 0.5^2, 0.01^2]);   % Propagation 用的车辆状态噪声
R = diag([1, (0.12)^2]);             % 测量噪声协方差

%% 为使 SLAM_PF 调用到正确的测量函数，在 main 中定义 Measurement 包装函数
% 注意：此包装函数会覆盖 MATLAB 路径中已有的 Measurement.m（如果有），
% 保证 SLAM_PF 中调用 Measurement(veh, landmark) 实际返回 range-bearing 测量
Measurement = @(pos_vehicle, pos_landmark) Measurement_SLAM(pos_vehicle, pos_landmark, 'range_bearing');

%% 定义场景：真值
% 定义真 landmark 坐标（2×num_landmarks）
true_landmarks = [ -10,   0,  10,   5,  -5;
                   10,    5,  -5, -10,   0];

% 定义真车辆状态： [x; y; psi]，这里令车辆位于原点，航向 0 rad（朝 x 轴正方向）
true_vehicle = [0; 0; 0];

%% 模拟量测
% 对每个 landmark，利用 Measurement_SLAM（'range_bearing'）产生测量，
% 并添加符合 R 对角线协方差的噪声
meas_landmark = zeros(2, num_landmarks);
for l = 1:num_landmarks
    % 计算无噪声测量（[r; theta]）
    meas0 = Measurement_SLAM(true_vehicle, true_landmarks(:,l), 'range_bearing');
    % 添加噪声（高斯，均值为 0）
    noise = [sqrt(R(1,1))*randn; sqrt(R(2,2))*randn];
    meas_landmark(:,l) = meas0 + noise;
end

%% 设置视野指示：假设所有 landmark 均在视野内
index_fov = ones(1, num_landmarks);

%% 初始化粒子（调用 Initialisation_particles.m）
% 假设初始车辆状态与真值接近，初始 landmark 估计偏差较大
initial_vehicle = true_vehicle + [0.5*randn; 0.5*randn; 0.05*randn];
% 初始 landmark 估计：在真值基础上加上较大偏差
initial_landmarks = true_landmarks + 2*randn(2, num_landmarks);
% 调用提供的 Initialisation_particles.m (确保此文件在路径中)
particles = Initialisation_particles(initial_vehicle, initial_landmarks);

%% 调用粒子滤波 SLAM 更新（SLAM_PF.m）
% SLAM_PF 内部调用 Measurement（本例中会调用我们在 main 中定义的包装函数）
particles = SLAM_PF(particles, meas_landmark, index_fov);

%% 对所有粒子中各 landmark 估计取均值作为最终估计
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
disp('粒子滤波 SLAM 估计（取均值）:');
disp(est_landmarks);

%% 绘图对比
figure; hold on; grid on;
plot(true_landmarks(1,:), true_landmarks(2,:), 'bo', 'MarkerSize',10, 'LineWidth',2);
plot(est_landmarks(1,:), est_landmarks(2,:), 'rx', 'MarkerSize',10, 'LineWidth',2);
legend('真值 Landmark', '估计 Landmark');
xlabel('x (m)'); ylabel('y (m)');
title('基于 Range-Bearing 测量的粒子滤波 SLAM 结果');
