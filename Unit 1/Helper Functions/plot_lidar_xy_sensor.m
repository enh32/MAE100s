function plot_lidar_xy_sensor(maxRange, angleData,distLidar, depthXY, lidar)
figure; 
hold on; %This keeps what has already been plotted if new plots are called
plot(0,0, 'bo'); %Plot sensor pose in sensor frame
plot(depthXY(1,:),depthXY(2,:),'c.'); % Plot the data
axis equal % This line makes the x and y axis have the same scale
axis([-4 4 -4 4])

polar(angleData, distLidar,'k');
polar(angleData, maxRange*ones(1, length(angleData)),'r--'); %This is similar to plot, but uses polar coordinates. The 'r--' makes the line a red dashed line
polar([lidar(1,1),lidar(1,1),lidar(end,1),lidar(end,1)],[lidar(1,2),0,0,lidar(end,2)],'r--'); 

legend( 'Sensor Pose (in Sensor Frame)','Transformed LIDAR data', 'Wall', 'Sensor Footprint'); 
xlabel('x (m)')
ylabel('y (m)')
title('plot LIDAR  xy sensor')
set(gcf,'Visible','on'); 

end

