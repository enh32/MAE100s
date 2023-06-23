function plot_measurements(N, measurements, alpha, measurement,robotXY, map)
figure
h = subplot(1,2,1);
for i = 1:length(map)
    plot([map(i,1),map(i,3)],[map(i,2),map(i,4)], 'k');
    hold on;
end
plot(robotXY(1),robotXY(2),'ok')
axis([-2.5 2.5 -2.5 2.5])
grid on
axis equal
robotXY = robotXY+[0.13,0,0]';


% x = [repmat(robotXY(1),1,length(alpha)); ...
%      robotXY(1) + measurement'.*cos(alpha')];
% y = [repmat(robotXY(2),1,length(alpha)); ...
%      robotXY(2) + measurement'.*sin(alpha')];

x = [repmat(robotXY(1),1,length(alpha)); ...
     robotXY(1) + measurement'.*cos(alpha)];
y = [repmat(robotXY(2),1,length(alpha)); ...
     robotXY(2) + measurement'.*sin(alpha)];
h.ColorOrderIndex = 1;
plot(x,y,'-');
title('LIDAR Readings on Map'); 
h = subplot(1,2,2);



plot(measurements,'.-')
hold on
h.ColorOrderIndex = 1;
plot(repmat([1;N],1,length(alpha)),[mean(measurements);mean(measurements)],'--','linewidth',2)
h.ColorOrderIndex = 1;
plot(repmat([1;N],1,length(alpha)),[measurement';measurement'],'-','linewidth',2)
xlabel('measurements');
ylabel('distance measured');
title('LIDAR Measurements');
axis([1 N -2 5]);
end
