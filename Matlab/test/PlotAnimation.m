
load('KalmanLastCircle.mat');
%Animation
% Odom-Orange
ci1 = [255 204 153] ./ 255;
ce1  = [255, 128 , 0] ./ 255;
c1 = (ci1-ce1) ./ length(out.xOdo);

%Gps green
ci2 = [204 255 153] ./ 255;
ce2  = [0 255 0] ./ 255;
c2 = (ci2-ce2) ./ length(out.xGps);

% Kalman magenta
ci3 = [255 153 255] ./ 255;
ce3  = [204 0 204] ./ 255;
c3 = (ci3-ce3) ./ length(out.xKalman);

% Lidar blue
ci4 = [153 153 255] ./ 255;
ce4  = [0 0 255] ./ 255;
c4 = (ci4-ce4) ./ length(out.xLidar);


figure; set(gcf,'color','w');
hold on
xlim([-1.1 1.1]);
ylim([-1.1 1.1]);
axis square;
plot(out.xRef,out.yRef,'r','Linewidth',3);
pause(5);
for i = 2:length(out.xOdo)
    tic;
    %plot(out.xOdo(i-1:i), out.yOdo(i-1:i),'color',ci1-c1*i,'Linewidth',3);
    %plot(out.xLidar(i-1:i), out.yLidar(i-1:i),'color',ci4-c4*i,'Linewidth',3);
    plot(out.xKalman(i-1:i), out.yKalman(i-1:i),'color',ci3-c3*i,'Linewidth',3);
    plot(out.xGps(i-1:i), out.yGps(i-1:i),'color',ci2-c2*i,'Linewidth',2);  
    drawnow limitrate
    et=toc;
    while(et <= 0.1)
        et=toc;
    end
end
%legend("Reference","Odometry","Lidar","Kalman","GPS");
%grid
hold off

