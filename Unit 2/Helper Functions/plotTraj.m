function plotTraj(map, robotXYtraj, robotXYtrajNoisy)
figure
hold on; 
plot(robotXYtraj(1,:),robotXYtraj(2,:),'-r','linewidth',2)
plot(robotXYtrajNoisy(1,:),robotXYtrajNoisy(2,:),'-r')
axis([-2.5 2.5 -2.5 2.5])
title('Noisy Robot Trajectory'); 

for i = 1:length(map)
    plot([map(i,1),map(i,3)],[map(i,2),map(i,4)], 'k');
end

legend('Trajectory - no noise', 'Trajectory - noise'); 
grid on
axis equal
set(gcf,'Visible','on')

end