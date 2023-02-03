maxWheelSpeed = 10;
K = 1;
pose_begin = [0 0 0];
xg = [0 3 -3 -3];
yg = [3 3 -3 -0.1];

epsilon = 0.1;
f = figure(5);
set(f,'position',[10,10,550,550])
ax = axes('Parent',f,'position',[0.13 0.25  0.77 0.67]);
hold on
axis([-4 4 -4 4])
axis equal

plot(pose_begin(1),pose_begin(2),'ko','MarkerSize',10);
temp = transform_points_OLD(pose_begin,[.25 0]);
plot([pose_begin(1) temp(1)],[pose_begin(2) temp(2)],'k','linewidth',2);
plot(xg,yg,'ro');

a = getTraj(epsilon);

for i = 1:length(a)
    traj = a(i).pose_history;
    h(i) = plot(traj(:,1),traj(:,2),'b');
end
    

b = uicontrol('Parent',f,'Style','slider','Position',[81,54,419,23],...
              'value',0.1, 'min',0.1, 'max',2);
bgcolor = f.Color;
bl1 = uicontrol('Parent',f,'Style','text','Position',[50,54,23,23],...
                'String','0.1','BackgroundColor',bgcolor);
bl2 = uicontrol('Parent',f,'Style','text','Position',[500,54,23,23],...
                'String','2','BackgroundColor',bgcolor);
bl3 = uicontrol('Parent',f,'Style','text','Position',[240,25,100,23],...
                'String','epsilon','BackgroundColor',bgcolor);
            
addlistener(b,'ContinuousValueChange',@(hObject, event) updatePlot(hObject, event,h));

function updatePlot(hObject,event,h)
    eps = hObject.Value;
    a = getTraj(eps);
    for i = 1:length(a)
        updated_traj = a(i).pose_history;
        h(i).XData = updated_traj(:,1);
        h(i).YData = updated_traj(:,2);
    end
    drawnow
end


function [a] = getTraj(epsilon)

maxWheelSpeed = 10;
K = 1;
pose_begin = [0 0 0];
xg = [0 3 -3 -3];
yg = [3 3 -3 -0.1];

for j = 1:length(xg)
    goal_xy = [xg(j),yg(j)];
    pose_current = pose_begin;
    a(j).time = 0;
    dt = 0.01;
    closeEnough = .05;
    a(j).pose_history = pose_current;
    while norm(pose_current(1:2) - goal_xy) > closeEnough && a(j).time < 30

        % Calculate commands
        Vxy_global = K.*(goal_xy - pose_current(1:2));
        [V,omega] = feedbackLin(Vxy_global(1),Vxy_global(2),pose_current(3),epsilon);
        [V,omega] = limitCommands(V,omega,maxWheelSpeed);

        % Integrate kinematics
        pose_current = integrateDiffDrive(pose_current, V, omega, dt);
        pose_current = pose_current(end,:);
        a(j).pose_history = [a(j).pose_history; pose_current];
        a(j).time = a(j).time + dt;

    end
end
end