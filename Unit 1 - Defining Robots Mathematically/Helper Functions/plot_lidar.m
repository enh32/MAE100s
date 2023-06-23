function plot_lidar(angleData,distLidar)
figure; 
plot(angleData, distLidar); 
xlabel('angle (rad)')
ylabel('range (m)')
axis([-2.5 2.5 0 4.1])
title('plot LIDAR range')
set(gcf,'Visible','on'); 

end

