clear s
dT = 0.02;
lidarUpdatePeriod = 0.2;
lidarCounter = 0;
lx = 0.1575;
radius = 0.05;
s.x = zeros(10,1);
s.A = [1  0  0  dT  0  0;
       0  1  0  0  dT  0;
       0  0  1  0   0 dT;
       0  0  0  0   0  0;
       0  0  0  0   0  0;
       0  0  0  0   0  0];
   
% Define a process noise (stdev) of 2 volts as the car operates:
odomDev = 0.5; % variance, hence stdev^2
dT2 = (dT*dT)/2.0 * odomDev;
dT3 = (dT*dT*dT)/3.0 * odomDev;
s.Q = [dT3  0    0   dT2   0    0;
        0  dT3   0    0   dT2   0;
        0   0   dT3   0    0   dT2;
       dT2  0    0   dT    0    0;
        0  dT2   0    0   dT    0;
        0   0   dT2   0    0   dT];
% Define the voltimeter to measure the voltage itself:
s.H = [0 0 1 0 0 0;
       1 0 0 0 0 0;
       0 1 0 0 0 0;
       0 0 1 0 0 0];
% Define a measurement error (stdev)
n1 = 0.001; n2 = 0.03; n3=0.03; n4=0.005;
s.R = [n1*n1    0     0     0;
         0    n2*n2   0     0;
         0      0   n3*n3   0;
         0      0     0   n4*n4];
% Define system input (control) functions (constant speed)
r4 = radius / 4.0;
r4l = r4 / (lx*2);
s.B = [ 0      0    0     0;
        0      0    0     0;
        0      0    0     0;
       -r4    r4    r4  -r4;
       r4     r4    r4   r4;
       r4l   -r4l  r4l  -r4l];

w1=0.5; w2=0.5; w3=0.5; w4=0.5;
s.u = [ w1;
        w2;
        w3;
        w4];
% Specify an initial state:
s.x = zeros(6,1);
s.P = eye(6);

% Generate random voltages and watch the filter operate.
truX=[]; 
truY=[];
truZ=[];
imu = [];
lidarX=[];
lidarY=[];
lidarZ=[];
posX = [];
posY = [];
thetaZ = [];
timeUpdate = []; time = [];
truY(1) = 0;truX(1) = 0;truZ(1) = 0;
for t=0:dT:5
   truX = [truX; truX(end)+r4*(-w1 +w2 +w3 -w4)*dT];
   truY = [truY; truY(end)+r4*(+w1 +w2 +w3 +w4)*dT];
   truZ = [truZ; truZ(end)+r4l*(+w1 -w2 +w3 -w4)*dT];
   if(lidarCounter >= (lidarUpdatePeriod/dT))
       imu =    [imu;       truZ(end) + randn*0.01]; % simulate IMU orientation Z
       lidarX = [lidarX;    truX(end) + randn*0.05]; % simulate lidar position X
       lidarY = [lidarY;    truY(end) + randn*0.05]; % simulate lidar position Y
       lidarZ = [lidarZ;    truZ(end) + randn*0.05]; % simulate lidar orientation Z
       timeUpdate = [timeUpdate, t];
       s(end).z(1) = imu(end);
       s(end).z(2) = lidarX(end);
       s(end).z(3) = lidarY(end);
       s(end).z(4) = lidarZ(end);    
       s(end+1) = kalmanUpdate(s(end));
       lidarCounter = 1;
   else
       s(end+1) = kalmanPredict(s(end));
       lidarCounter = lidarCounter +1; 
   end
   posX = [posX; s(end).x(1)];
   posY = [posY; s(end).x(2)];
   thetaZ = [thetaZ; s(end).x(3)];
   time = [time; t];
end


figure
hold on
grid on
% plot measurement data:
hzX=plot(timeUpdate,lidarX,'r.');
% plot a-posteriori state estimates:
hkX=plot(time,posX,'b-');
htX=plot(time,truX(2:end),'g-');
legend([hzX hkX htX],'observations','Kalman output','True Position')
title('Position X')
hold off

figure
hold on
grid on
% plot measurement data:
hzY=plot(timeUpdate,lidarY,'r.');
% plot a-posteriori state estimates:
hkY=plot(time,posY,'b-');
htY=plot(time,truY(2:end),'g-');
legend([hzY hkY htY],'observations','Kalman output','True Position')
title('Position Y')
hold off

figure
hold on
grid on
% plot measurement data:
hzZ=plot(timeUpdate,imu,'r.');
hzZL=plot(timeUpdate,lidarZ,'magenta*');
% plot a-posteriori state estimates:
hkZ=plot(time,thetaZ,'b-');
htZ=plot(time,truZ(2:end),'g-');
legend([hzZ hzZL hkZ htZ],'observations IMU','observations Lidar','Kalman output','True Orientation')
title('Orientation Z')
hold off
