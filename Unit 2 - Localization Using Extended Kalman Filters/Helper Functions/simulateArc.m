function [u, robotXYtraj, robotXYtrajNoisy, measurements] = simulateArc(robot_pose, goal_xy, maxWheelSpeed, ekfArgs)
[V,omega,time] = findArcPath(robot_pose',goal_xy',maxWheelSpeed);
t = 0:ekfArgs.dt:time;
u = [V*ones(1,length(t));omega*ones(1,length(t))];
robotXYtraj = zeros(3,1+length(u));
robotXYtrajNoisy = zeros(3,1+length(u));
measurements = zeros(2,length(u));
robotXYtraj(:,1) = robot_pose;
robotXYtrajNoisy = robot_pose;
for i = 1:length(u)
    
    robotXYtraj(:,i+1) = [robotXYtraj(1,i) + u(1,i)*cos(robotXYtraj(3,i))*ekfArgs.dt; ...
        robotXYtraj(2,i) + u(1,i)*sin(robotXYtraj(3,i))*ekfArgs.dt; ...
        robotXYtraj(3,i) + u(2,i)*ekfArgs.dt];       
    
    robotXYtrajNoisy(:,i+1) = [robotXYtrajNoisy(1,i) + u(1,i)*cos(robotXYtrajNoisy(3,i))*ekfArgs.dt; ...
        robotXYtrajNoisy(2,i) + u(1,i)*sin(robotXYtrajNoisy(3,i))*ekfArgs.dt; ...
        robotXYtrajNoisy(3,i) + u(2,i)*ekfArgs.dt] +  ...
        sqrt(ekfArgs.R)*randn(3,1)*ekfArgs.dt;
    
    measurements(:,i) = robotXYtrajNoisy(1:2,i) + ekfArgs.Q*randn(2,1);   
end
end