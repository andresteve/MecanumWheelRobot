
load('KalmanLast.mat');


% RMS Error
rmsOdoGpsX = sqrt(mean((out.xGps - out.xOdo).^2));
rmsOdoGpsY = sqrt(mean((out.yGps - out.yOdo).^2));
rmsLidarGpsX = sqrt(mean((out.xGps - out.xLidar).^2));
rmsLidarGpsY = sqrt(mean((out.yGps - out.yLidar).^2));
rmsKalmanGpsX = sqrt(mean((out.xGps - out.xKalman).^2));
rmsKalmanGpsY = sqrt(mean((out.yGps - out.yKalman).^2));

orange = [255, 128 , 0] ./255;

%Position
figure;  set(gcf,'color','w');
subplot(2,2,1);
hold on;
plot(out.xRef,out.yRef,"Color","red","LineWidth",2);
plot(out.xOdo,out.yOdo,"Color",orange,"LineWidth",2);
plot(out.xKalman,out.yKalman,"Color","magenta","LineWidth",1.5);
plot(out.xGps,out.yGps,"Color","green","LineWidth",1);
title('Odometry Position');
xlabel('X [m]'); ylabel('Y [m]'); xlim([-0.8 0.8]); ylim([-0.8 0.8]);
legend('Reference', 'Odometry', 'Kalman', 'Gps');
grid
hold off;
subplot(2,2,2);
hold on;
plot(out.xRef,out.yRef,"Color","red","LineWidth",2);
plot(out.xOdo,out.yOdo,"Color",orange,"LineWidth",2);
plot(out.xGps,out.yGps,"Color","green","LineWidth",1.5);
title('Odometry Position');
xlabel('X [m]'); ylabel('Y [m]'); xlim([-0.8 0.8]); ylim([-0.8 0.8]);
axis square
legend('Reference', 'Odometry', 'Gps');
grid
hold off
subplot(2,2,3);
hold on;
plot(out.xRef,out.yRef,"Color","red","LineWidth",2);
plot(out.xLidar,out.yLidar,"Color","blue","LineWidth",2);
plot(out.xGps,out.yGps,"Color","green","LineWidth",1.5);
title('Lidar Position');
xlabel('X [m]'); ylabel('Y [m]'); xlim([-0.8 0.8]); ylim([-0.8 0.8]);
legend('Reference', 'Lidar', 'Gps');
grid
hold off
subplot(2,2,4);
hold on;
xCategory = categorical({'Odometry-Gps','Lidar-Gps', 'Kalman-Gps'});
yValues = [rmsOdoGpsX rmsOdoGpsY; rmsLidarGpsX rmsLidarGpsY; rmsKalmanGpsX rmsKalmanGpsY; ];
bar(xCategory,yValues)
title('RMS Error [m]');
grid
hold off;

%Position in time
figure;  set(gcf,'color','w');
subplot(2,1,1);
hold on;
plot(out.t,out.xOdo(1:length(out.t)),"Color",orange,"LineWidth",2);
plot(out.t,out.xLidar(1:length(out.t)),"Color","blue","LineWidth",2);
plot(out.t,out.xKalman(1:length(out.t)),"--","Color","magenta","LineWidth",2);
plot(out.t,out.xGps(1:length(out.t)),"Color","green","LineWidth",1.5);
title('X Position');
xlabel('Time [s]'); ylabel('Position [m]'); ylim([-1.1 1.1]);
legend('Odometry','Lidar', 'Kalman', 'Gps');
grid
hold off;
subplot(2,1,2);
hold on;
plot(out.t,out.yOdo(1:length(out.t)),"Color",orange,"LineWidth",2);
plot(out.t,out.yLidar(1:length(out.t)),"Color","blue","LineWidth",2);
plot(out.t,out.yKalman(1:length(out.t)),"--","Color","magenta","LineWidth",2);
plot(out.t,out.yGps(1:length(out.t)),"Color","green","LineWidth",1.5);
title('Y Position');
xlabel('Time [s]'); ylabel('Position [m]'); ylim([-1.1 1.1]);
legend('Odometry', 'Lidar', 'Kalman', 'Gps');
grid
hold off


%Robot Speed
figure;  set(gcf,'color','w');
hold on
subplot(2,1,1);
plot(out.t,out.speedX,"Color","red","LineWidth",1.5);
title('Speed on X-axis');
xlabel('Time [s]');
ylabel('Speed [m/s]');
grid
subplot(2,1,2);
plot(out.t,out.speedY,"Color","red","LineWidth",1.5);
title('Speed on Y-axis');
xlabel('Time [s]');
ylabel('Speed [m/s]');
grid
hold off

% Wheel Speed
figure;  set(gcf,'color','w');
subplot(2,2,1);
hold on
plot(out.t,out.w1Ref(1:length(out.t)),"Color","red","LineWidth",1.5);
plot(out.t,out.w1(1:length(out.t)),"Color","green","LineWidth",1.5);
legend("Reference","Speed");
title('Speed Front Right Wheel [1]');
xlabel('Time [s]');
ylabel('Speed [rad/s]');
grid
hold off
subplot(2,2,2);
hold on;
plot(out.t,out.w2Ref(1:length(out.t)),"Color","red","LineWidth",1.5);
plot(out.t,out.w2(1:length(out.t)),"Color","green","LineWidth",1.5);
legend("Reference","Speed");
title('Speed Front Left Wheel [2]');
xlabel('Time [s]');
ylabel('Speed [rad/s]');
grid
hold off;
subplot(2,2,3);
hold on;
plot(out.t,out.w3Ref(1:length(out.t)),"Color","red","LineWidth",1.5);
plot(out.t,out.w3(1:length(out.t)),"Color","green","LineWidth",1.5);
legend("Reference","Speed");
title('Speed Rear Right Wheel [3]');
xlabel('Time [s]');
ylabel('Speed [rad/s]');
grid
hold off;
subplot(2,2,4);
hold on;
plot(out.t,out.w4Ref(1:length(out.t)),"Color","red","LineWidth",1.5);
plot(out.t,out.w4(1:length(out.t)),"Color","green","LineWidth",1.5);
legend("Reference","Speed");
title('Speed Rear Left Wheel [4]');
xlabel('Time [s]');
ylabel('Speed [rad/s]');
grid
hold off


%Angle
figure;  set(gcf,'color','w');
hold on
plot(out.t,out.odomThetaZ(1:length(out.t)),"Color","red","LineWidth",2);
plot(out.t,out.IMU(1:length(out.t)),"Color","cyan","LineWidth",2);
plot(out.t,out.lidarThetaZ(1:length(out.t)),"Color","blue","LineWidth",2);
plot(out.t,out.kalmanThetaZ(1:length(out.t)),"--","Color","magenta","LineWidth",2);
title('Robot orientation');
xlabel('Time [s]');
ylabel('Orientation [rad]');
legend('Odometry','IMU', 'Lidar', 'Kalman Filter');
%legend('IMU');
grid
hold off

