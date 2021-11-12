function plot_frames2(sensor_pose, robot_pose, dataxy_robot, dataxy_global, plot_title)

radius = 0.13;
lim = max(abs(dataxy_global(:)));
lim = max([max(abs(dataxy_robot(:))),lim]);
lim = ceil(lim);
figure
subplot(1,2,1)
title('Robot Frame')

hold on
grid on
axis equal
axis([-lim lim -lim lim])
quiver(sensor_pose(1),sensor_pose(2),radius*cos(sensor_pose(3)),radius*sin(sensor_pose(3)),0,'filled','s')
th = linspace(0,2*pi,30);
plot(radius*cos(th),radius*sin(th),'k-')
plot([0,radius], ...
    [0,0],'k-')
plot(dataxy_robot(:,1),dataxy_robot(:,2),'r.','markersize',10)
subplot(1,2,2)
title('Global Frame')

hold on
grid on
axis equal
axis([-lim lim -lim lim])
sensor_coord = transform_points(robot_pose,sensor_pose(1:2));
quiver(sensor_coord(1),sensor_coord(2), ...
    radius*cos(sensor_pose(3)+robot_pose(3)),radius*sin(sensor_pose(3)+robot_pose(3)), ...
    0,'filled','s')
plot(robot_pose(1)+radius*cos(th),robot_pose(2)+radius*sin(th),'k-')
plot([robot_pose(1),robot_pose(1)+radius*cos(robot_pose(3))], ...
    [robot_pose(2),robot_pose(2)],'k-')
plot(dataxy_global(:,1),dataxy_global(:,2),'r.','markersize',10)
sgtitle(plot_title)
end