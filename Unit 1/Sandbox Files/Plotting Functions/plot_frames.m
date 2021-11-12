function plot_frames(robot_pose, dataxy_robot, dataxy_global, plot_title)

radius = 0.13;
lim = max(abs(dataxy_robot(:)));
lim = max([max(abs(dataxy_global(:))),lim]);
lim = ceil(lim);
figure
subplot(1,2,1)
title('Robot Frame')

hold on
grid on
axis equal
axis([-lim lim -lim lim])
% quiver(0,0,radius,0,0,'filled','s')
th = linspace(0,2*pi,30);

plot(radius*cos(th),radius*sin(th),'k-')
plot([0,radius], ...
    [0,0],'k-')

plot(dataxy_robot(1,:),dataxy_robot(2,:),'r.','markersize',10)
subplot(1,2,2)
title('Global Frame')

hold on
grid on
axis equal
axis([-lim lim -lim lim])


plot(robot_pose(1)+radius*cos(th),robot_pose(2)+radius*sin(th),'k-')

plot([robot_pose(1),robot_pose(1)+radius*cos(robot_pose(3))], ...
    [robot_pose(2),robot_pose(2)+radius*sin(robot_pose(3))],'k-')

% plot([-robot_pose(1),-robot_pose(1)+radius*cos(-robot_pose(3))], ...
%     [-robot_pose(2),-robot_pose(2)+radius*sin(-robot_pose(3))],'k-')

plot(dataxy_global(1,:),dataxy_global(2,:),'r.','markersize',10)
sgtitle(plot_title)
end
