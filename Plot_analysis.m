%% vehicle Trajectory
figure; hold on;
title('vehicle Trajectories');
plot(real_vehicle1(1,:),real_vehicle1(2,:),'r');
plot(real_vehicle2(1,:),real_vehicle2(2,:),'b');
plot(pos_vehicle1(1,:),pos_vehicle1(2,:),'r:');
plot(pos_vehicle2(1,:),pos_vehicle2(2,:),'b:');
grid on; xlabel('x (m)'); ylabel('y (m)');
axis([-30, 30, -30, 30]);
legend('True vehicle1','True vehicle2','Estimated vehicle1','Estimated vehicle2');

%% vehicle Position Error
error_vehicle1 = pos_vehicle1-real_vehicle1;
error_vehicle2 = pos_vehicle2-real_vehicle2;
rmse_vehicle1 = sqrt(sum(error_vehicle1.^2,1));
rmse_vehicle2 = sqrt(sum(error_vehicle2.^2,1));

figure; 
set(gcf,'position',[0 0 500 1000]);
subplot(411); hold on;
title('vehicle Estimation Error');
plot(time,rmse_vehicle1,'r');
plot(time,rmse_vehicle2,'b');
grid on; xlabel('Time (s)'); ylabel('RMSE [m]'); legend('vehicle1','vehicle2');

disp(['Average vehicle1 Position Estimation Error: ',num2str(mean(rmse_vehicle1))]);
disp(['Average vehicle2 Position Estimation Error: ',num2str(mean(rmse_vehicle2))]);

%% Landmark Position Error
error_landmark1 = pos_landmark1-real_landmarks;
error_landmark2 = pos_landmark2-real_landmarks;
error_landmark1 = reshape(error_landmark1(:),[2*num_landmarks,timesteps]);
error_landmark2 = reshape(error_landmark2(:),[2*num_landmarks,timesteps]);
rmse_landmark1 = sqrt(sum(error_landmark1.^2,1));
rmse_landmark2 = sqrt(sum(error_landmark2.^2,1));

subplot(412); hold on; 
title('Landmark Estimation Error');
plot(time,rmse_landmark1,'r');
plot(time,rmse_landmark2,'b');
grid on; xlabel('Time (s)'); ylabel('RMSE [m]'); legend('vehicle1','vehicle2');


disp(['Average Landmark Estimation Error from vehicle1: ',num2str(mean(rmse_landmark1))]);
disp(['Average Landmark Estimation Error from vehicle2: ',num2str(mean(rmse_landmark2))]);

if fusion_flag>0
    error_landmark_SF = pos_landmark_SF-real_landmarks;
    error_landmark_SF = reshape(error_landmark_SF(:),[2*num_landmarks,timesteps]);
    rmse_landmark_SF = sqrt(sum(error_landmark_SF.^2,1));
    plot(time,rmse_landmark_SF,'g');
    legend('vehicle1','vehicle2','Fusion');
    disp(['Average Landmark Estimation Error from Fusion: ',num2str(mean(rmse_landmark_SF))]);
end

%% Covariance Matrix
cov_landmark1 = reshape(cov_landmark1(:),[4*num_landmarks,timesteps]);
cov_landmark2 = reshape(cov_landmark2(:),[4*num_landmarks,timesteps]);
sum_cov_landmark1 = sqrt(sum(cov_landmark1.^2,1));
sum_cov_landmark2 = sqrt(sum(cov_landmark2.^2,1));

subplot(413); hold on; 
title('Landmark Covariance');
plot(time,sum_cov_landmark1,'r');
plot(time,sum_cov_landmark2,'b');
grid on; xlabel('Time (s)'); ylabel('Covariance [m^2]'); legend('vehicle1','vehicle2');

if fusion_flag>0
    cov_landmark_SF = reshape(cov_landmark_SF(:),[4*num_landmarks,timesteps]);
    sum_cov_landmark_SF = sqrt(sum(cov_landmark_SF.^2,1));
    plot(time,sum_cov_landmark_SF,'g');
    legend('vehicle1','vehicle2','Fusion');
end

%% Number of Landmarks within FOV
subplot(414); hold on;
title('Number of landmarks within FOV');
plot(time,squeeze(sum(index_fov(1,:,:),2)),'r');
plot(time,squeeze(sum(index_fov(2,:,:),2)),'b');
grid on; xlabel('Time (s)'); ylabel('No. of landmarks'); legend('vehicle1','vehicle2');

if fusion_flag>0
    plot(time,squeeze(sum(index_fov(3,:,:),2)),'g');
    legend('vehicle1','vehicle2','Fusion');
end