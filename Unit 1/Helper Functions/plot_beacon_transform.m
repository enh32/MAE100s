function  plot_beacon_transform(tags, dataxy_global, robot_pose, robotRadius)
figure; 
hold on ;
for i = 1:size(tags,1)  
    beacon_number = round(tags(i,2));
    plot(dataxy_global(1, i),dataxy_global(2, i),'ro','Markersize',5);
    text(dataxy_global(1, i)+.1,dataxy_global(2, i),num2str(beacon_number));
end
th = linspace(0,2*pi,30);
plot(robot_pose(1)+robotRadius*cos(th),robot_pose(2)+robotRadius*sin(th),'k-')
plot([robot_pose(1),robot_pose(1)+robotRadius*cos(robot_pose(3))], ...
    [robot_pose(2),robot_pose(2)+robotRadius*sin(robot_pose(3))],'k-')

axis([-5 5 -5 5])
title('plot beacon global')
set(gcf,'Visible','on'); 

end

