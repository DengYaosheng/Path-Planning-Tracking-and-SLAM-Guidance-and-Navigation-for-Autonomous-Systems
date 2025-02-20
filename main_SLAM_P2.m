%% main_SLAM_P2.m
% 本脚本用于测试 Task 2 P2，即使用仅有方位角量测的粒子滤波 SLAM。
% 该脚本利用 SLAM_PF_bearing.m 及 Measurement_SLAM.m（测量类型为 'bearing_only'），
% 初始化场景、生成带噪声的方位角量测，然后更新粒子，最后对所有粒子的路标估计取均值并绘图。


%% 设置全局变量
global num_landmarks num_particles Q R_bearing
num_landmarks = 5;
num_particles = 50;
% 车辆状态噪声（用于 Propagation.m），车辆状态为 [x; y; psi]
Q = diag([0.5^2, 0.5^2, 0.01^2]);
% 量测噪声（仅方位角）：例如方差设为 (0.12)^2
R_bearing = (0.12)^2;

%% 定义场景：真值
% 定义 5 个路标的真值坐标（2×num_landmarks）
true_landmarks = [ -10,   0,  10,   5,  -5;
                   10,    5,  -5, -10,   0];

% 定义真车辆状态 [x; y; psi]，车辆位于原点，航向 0 rad（朝 x 轴正方向）
true_vehicle = [0; 0; 0];

%% 生成仅有方位角的量测
% 对每个路标，调用 Measurement_SLAM 生成 'bearing_only' 量测，然后加上高斯噪声
meas_landmark_bearing = zeros(1, num_landmarks);
for l = 1:num_landmarks
    meas0 = Measurement_SLAM(true_vehicle, true_landmarks(:,l), 'bearing_only');
    noise = sqrt(R_bearing) * randn;
    meas_landmark_bearing(l) = meas0 + noise;
end

%% 设置视野指示：假设所有路标均在视野内
index_fov = ones(1, num_landmarks);

%% 初始化粒子（调用 Initialisation_particles.m）
% 初始车辆状态稍有偏差，初始路标估计误差较大
initial_vehicle = true_vehicle + [0.5*randn; 0.5*randn; 0.05*randn];
initial_landmarks = true_landmarks + 2 * randn(2, num_landmarks);
particles = Initialisation_particles(initial_vehicle, initial_landmarks);

%% 调用 SLAM_PF_bearing.m 更新粒子（利用 bearing-only 量测）
particles = SLAM_PF_bearing(particles, meas_landmark_bearing, index_fov);

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
disp('粒子滤波 SLAM 估计（取均值）:');
disp(est_landmarks);

%% 绘图对比
figure; hold on; grid on;
plot(true_landmarks(1,:), true_landmarks(2,:), 'bo', 'MarkerSize',10, 'LineWidth',2);
plot(est_landmarks(1,:), est_landmarks(2,:), 'rx', 'MarkerSize',10, 'LineWidth',2);
legend('真值 Landmark', '估计 Landmark');
xlabel('x (m)'); ylabel('y (m)');
title('仅 Bearing 量测的粒子滤波 SLAM 结果');
