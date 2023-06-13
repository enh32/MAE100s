function plotFeedbackLin(goal_xy,pose_begin,epsilon,maxWheelSpeed,K, plot_title)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
% Other initialized parameters, no need to change
dt = 0.05;
closeEnough = .05;
time = 0;

% Variables needed for the loop
pose_current = pose_begin;
pose_history = pose_current;

% Set up the plotting
f = figure;
set(gcf,'Visible','on')
p = plot(pose_history(1,:),pose_history(2,:));
hold on
title(plot_title); 
plot(goal_xy(1),goal_xy(2),'ro','MarkerSize',10)
if exist('h1','var')
    delete([h1,h2])
    clear h1 h2
end
h1 = plot(pose_current(1),pose_current(2),'ko','MarkerSize',10);
temp = transform_points(pose_current,[.05; 0]);
h2 = plot([pose_current(1) temp(1)],[pose_current(2) temp(2)],'k','linewidth',2);
axis equal
if exist('ht' ,'var')
    delete(ht)
    clear ht
end
ht = text(-1,1,['time = ' num2str(time) 's']);

% axis([-1.5 1.5 -1.5 1.5])


% Feedback Control Loop
tic
last = toc;
while norm(pose_current(1:2) - goal_xy) > closeEnough && toc < 30

    % Calculate commands
    Vxy_global = K.*(goal_xy - pose_current(1:2));
    [V,omega] = feedbackLin(Vxy_global(1),Vxy_global(2),pose_current(3),epsilon);
    [V,omega] = limitCommands(V,omega,maxWheelSpeed);

    % Integrate kinematics
    pose_current = integrateDiffDrive(pose_current, V, omega, dt);
    pose_current = pose_current(end,:)';
    pose_history = [pose_history pose_current];
    time = time + dt;
    
    % Update plots
    p.XData = pose_history(1,:);
    p.YData = pose_history(2,:);
    h1.XData = pose_current(1);
    h1.YData = pose_current(2);
    temp = transform_points(pose_current,[.05; 0]);
    h2.XData = [pose_current(1) temp(1)];
    h2.YData = [pose_current(2) temp(2)];
    ht.String = ['time = ' num2str(time) 's'];
    drawnow limitrate
    
    pause(dt-(toc-last))
    last = toc;

end

% Change goal marker color if reached
if norm(pose_current(1:2) - goal_xy) < closeEnough
    plot(goal_xy(1),goal_xy(2),'go','MarkerSize',10)
end
end
