%% main_EKF_SLAM.m
% 本脚本实现 EKF SLAM（Task 2 P4），利用 range & bearing 量测更新增强状态。
% 增强状态向量 x_aug = [x_v; y_v; psi_v; l1_x; l1_y; ...; lN_x; lN_y],
% 其中车辆状态为 [x_v; y_v; psi_v]，各路标为 [l_x; l_y].
% 车辆运动模型采用简单的 kinematics，路标假设静止。
% 量测模型为:
%    z = [ r; theta ] = [ sqrt((l_x-x_v)^2+(l_y-y_v)^2);
%                          atan2(l_y-y_v, l_x-x_v) ] + noise.
%
% 初始状态与协方差均设置一定误差，后续通过 EKF 预测与更新迭代估计车辆及路标状态。

clear; clc; close all;

%% 1. 加载仿真参数（调用 Initialisation_scenario.m）
% 此文件定义全局变量：dt, timesteps, real_vehicle1, real_landmarks, vel_cmd, 等
Initialisation_scenario; 
% 此处我们仅采用车辆1进行 EKF SLAM, 其真值轨迹保存在 real_vehicle1 中，
% 真路标坐标保存在 real_landmarks 中。

global dt num_landmarks Q R
% 本例中 Q 为 Propagation.m 使用的车辆状态噪声协方差，
% R 在本例中未直接使用，因为我们对 EKF 量测噪声使用 R_meas 定义。
% 设定路标总数与车辆噪声（可与 Initialisation_scenario 中保持一致）
% 注意: num_landmarks 已在 Initialisation_scenario.m 中定义
% 此外, 车辆控制命令 vel_cmd 已定义, 如 vel_cmd = [5; 0.1] （速度，角速率）

%% 2. 定义 EKF SLAM 的初始增强状态与协方差
% 增强状态: x_aug = [x_v; y_v; psi_v; l1_x; l1_y; ...; lN_x; lN_y]
% 初始化车辆状态采用真实初始状态加上少量误差
x_v0 = real_vehicle1(:,1) + [0.5*randn; 0.5*randn; 0.05*randn];
% 初始化路标估计：在真实值基础上加入较大噪声
initial_landmarks = real_landmarks + 2*randn(2, num_landmarks);
x_aug = [x_v0; reshape(initial_landmarks, num_landmarks*2, 1)];

% 初始协方差:
% 车辆部分：设定较小协方差（例如：0.1 m² for x,y; 0.01 for psi）
P_v0 = diag([0.1, 0.1, 0.01]);
% 路标部分：设定较大协方差（例如：5 m² 每个坐标）
P_l0 = kron(eye(num_landmarks), diag([5,5])); 
% 增强状态协方差矩阵：
P_aug = blkdiag(P_v0, P_l0);

n_aug = length(x_aug);  % 状态向量维度

%% 3. 设置 EKF 过程噪声与量测噪声
% 过程噪声只作用在车辆部分
Q_ekf = diag([0.5^2, 0.5^2, 0.01^2]);   % 车辆状态噪声（与 Q 保持一致）
% 量测噪声 R_meas for range-bearing
R_meas = diag([1, (0.12)^2]);

%% 4. 初始化存储变量
timesteps = length((1:timesteps));
est_vehicle = zeros(3, timesteps);
est_vehicle(:,1) = x_aug(1:3);
est_landmarks = zeros(2, num_landmarks, timesteps);
est_landmarks(:,:,1) = reshape(x_aug(4:end), 2, num_landmarks);

%% 5. EKF SLAM 主循环
for t = 2:timesteps
    % --- 预测步骤 ---
    % 车辆运动模型：假设车辆以常数速度 V = vel_cmd(1) 行驶，并以角速率 vel_cmd(2) 旋转
    V = vel_cmd(1);
    omega = vel_cmd(2);
    % 当前车辆状态:
    x_v = x_aug(1);
    y_v = x_aug(2);
    psi = x_aug(3);
    % 简单预测（使用前向欧拉积分）:
    x_v_pred = x_v + dt * V * sin(psi + dt*omega);
    y_v_pred = y_v + dt * V * cos(psi + dt*omega);
    psi_pred = psi + dt * omega;
    % 更新车辆部分预测结果:
    x_v_pred_vec = [x_v_pred; y_v_pred; psi_pred];
    
    % 构造车辆状态转移矩阵 F_v (3x3)
    F_v = eye(3);
    F_v(1,3) = dt * V * cos(psi + dt*omega);
    F_v(2,3) = -dt * V * sin(psi + dt*omega);
    
    % 构造增强状态转移矩阵 F (n_aug x n_aug)
    F = eye(n_aug);
    F(1:3,1:3) = F_v;
    % 注意：路标部分为静止（单位矩阵）
    
    % 过程噪声扩展到增强状态：
    Q_aug = zeros(n_aug);
    Q_aug(1:3,1:3) = Q_ekf;
    
    % 状态与协方差预测：
    x_aug = F * x_aug;
    P_aug = F * P_aug * F' + Q_aug;
    
    % --- 更新步骤 ---
    % 假设在当前时刻 t，车辆获得所有 num_landmarks 的 range-bearing 量测
    % 使用车辆1的真实状态（real_vehicle1(:,t)）与真路标生成带噪声的量测
    z_meas = zeros(2, num_landmarks);
    for j = 1:num_landmarks
        % 真实量测（无噪声）
        z_true = Measurement_SLAM(real_vehicle1(:,t), real_landmarks(:,j), 'range_bearing');
        % 加噪声：
        noise = [sqrt(R_meas(1,1))*randn; sqrt(R_meas(2,2))*randn];
        z_meas(:,j) = z_true + noise;
    end
    
    % 对每个路标进行更新
    for j = 1:num_landmarks
        % 当前车辆状态估计：
        x_v = x_aug(1);
        y_v = x_aug(2);
        % 当前路标 j 估计：位置索引：
        idx = 3 + 2*(j-1) + 1;
        lx = x_aug(idx);
        ly = x_aug(idx+1);
        
        % 预测量测：
        dx_val = lx - x_v;
        dy_val = ly - y_v;
        r_pred = sqrt(dx_val^2 + dy_val^2);
        theta_pred = atan2(dy_val, dx_val);
        z_pred = [r_pred; theta_pred];
        
        % 创新：
        innov = z_meas(:,j) - z_pred;
        % 包装角度误差到 [-pi, pi]
        innov(2) = mod(innov(2)+pi, 2*pi) - pi;
        
        % 计算量测雅可比矩阵 H_j 对于增强状态
        % H_j 为 2 x n_aug，只有车辆部分（1:3）和该路标部分（idx:idx+1）非零
        H_j = zeros(2, n_aug);
        % 对车辆状态部分：
        dr_dx = -dx_val / r_pred;
        dr_dy = -dy_val / r_pred;
        dr_dpsi = 0;
        dtheta_dx = dy_val / (r_pred^2);
        dtheta_dy = -dx_val / (r_pred^2);
        dtheta_dpsi = 0;
        H_j(:,1:3) = [dr_dx, dr_dy, dr_dpsi;
                      dtheta_dx, dtheta_dy, dtheta_dpsi];
        % 对路标 j 部分：
        dr_dlx = dx_val / r_pred;
        dr_dly = dy_val / r_pred;
        dtheta_dlx = -dy_val / (r_pred^2);
        dtheta_dly = dx_val / (r_pred^2);
        H_j(:, idx:idx+1) = [dr_dlx, dr_dly;
                             dtheta_dlx, dtheta_dly];
        
        % 创新协方差：
        S = H_j * P_aug * H_j' + R_meas;
        % Kalman 增益：
        K = P_aug * H_j' / S;
        
        % 更新增强状态和协方差：
        x_aug = x_aug + K * innov;
        P_aug = (eye(n_aug) - K * H_j) * P_aug;
    end
    
    % 存储当前车辆状态与路标估计（用于后续绘图）
    est_vehicle(:,t) = x_aug(1:3);
    est_landmarks(:,:,t) = reshape(x_aug(4:end), 2, num_landmarks);
end

%% 6. 绘制估计结果
figure; 
hold on; grid on; axis equal;
% 绘制真实车辆轨迹（车辆1的真实轨迹）
plot(real_vehicle1(1,:), real_vehicle1(2,:), 'b-', 'LineWidth',2, 'DisplayName','真实车辆');
% 绘制 EKF 估计车辆轨迹
plot(est_vehicle(1,:), est_vehicle(2,:), 'r--', 'LineWidth',2, 'DisplayName','估计车辆');
% 绘制真实路标
plot(real_landmarks(1,:), real_landmarks(2,:), 'bo', 'MarkerSize',10, 'LineWidth',2, 'DisplayName','真实路标');
% 绘制 EKF 估计路标（最后时刻）
est_l_final = est_landmarks(:,:,end);
plot(est_l_final(1,:), est_l_final(2,:), 'rx', 'MarkerSize',10, 'LineWidth',2, 'DisplayName','估计路标');
legend('show');
xlabel('x (m)'); ylabel('y (m)');
title('EKF SLAM (Range & Bearing Measurements)');

%% 绘制车辆位置误差随时间
veh_error = sqrt(sum((est_vehicle(1:2,:) - real_vehicle1(1:2,:)).^2, 1));
figure;
plot((1:timesteps)*dt, veh_error, 'r-', 'LineWidth',2);
grid on; xlabel('时间 (s)'); ylabel('位置误差 (m)');
title('EKF SLAM 车辆位置误差');
