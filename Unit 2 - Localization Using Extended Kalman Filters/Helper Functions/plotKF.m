function plotKF(mu,sigma,pose,map,measFunc, varargin)

if size(pose,1)==2
    pose(3,:) = 0;
end

N = size(sigma,3);
lim = ceil(max(abs(map(:)))+1);
epsilon = 1;
f = figure;
set(f,'Units','normal','position',[.1 .1 .8 .8])
ax = axes('Parent',f,'position',[0.13 0.25  0.77 0.67]);
hold on
axis([-lim lim -lim lim])
axis equal

for i = 1:length(map)
    plot(ax,[map(i,1),map(i,3)],[map(i,2),map(i,4)], 'k');
    hold on;
end

if nargin > 5
    beacons = varargin{1};
    plot(beacons(:,1),beacons(:,2),'.m','markersize',20)
end

% plot(pose(1,1),pose(2,1),'ko','MarkerSize',10);
plot(pose(1,:),pose(2,:),'k-')
th = linspace(pose(3,1),2*pi+pose(3,1),25);

hr = plot([(pose(1,1)+0.13*cos(th)) pose(1,1)],[(pose(2,1)+0.13*sin(th)) pose(2,1)],'k','linewidth',1);
% plotCovEllipse(mu(1:2,1), sigma(1:2,1:2,1),1,[{'color'},{'g'},{'linestyle'},{'-'},{'linewidth'},{1}]);
hold on
plot(mu(1,:),mu(2,:),'r-');

% he = plotCovEllipse(mu(:,end), sigma(:,:,end),1,[{'color'},{'k'},{'linestyle'},{'-'},{'linewidth'},{1}]);
h = plotCovEllipse(mu(1:2,1), sigma(1:2,1:2,1),1,[{'color'},{'r'},{'linestyle'},{'-'},{'linewidth'},{1}]);
h2 = plot(mu(1,1),mu(2,1),'r.','MarkerSize',15);
if size(mu,1) == 3
    txt = strcat('mu= ','[', num2str(mu(1,1)), ',', num2str(mu(2,1)),',',num2str(mu(3,1)),']');
elseif size(mu,1) == 2
    txt = strcat('mu= ','[', num2str(mu(1,1)), ',', num2str(mu(2,1)),']');
end
text(mu(1,1),mu(2,1),txt)

t = text(-lim+0.5,lim-0.5,0,num2str(0));
axis([-lim lim -lim lim])
data.hr = hr;
data.t = t;
data.h = h;
data.h2 = h2;
data.mu = mu;
data.sigma = sigma;
data.pose = pose;
title("EKF: " + func2str(measFunc)); 

b2 = uicontrol('Parent',f,'Units','normal','Position',[253/540,510/540,50/540,30/540],...
              'String','Run KF');
b2.Callback = @runKF;

b = uicontrol('Parent',f,'Style','slider','Units','normal','Position',[.15,54/540,419/540,23/540],...
              'value',1, 'min',1, 'max',N,'SliderStep',[1/(N-1),0.1]);
bgcolor = f.Color;
bl1 = uicontrol('Parent',f,'Style','text','Units','normal','Position',[50/540,54/540,23/540,23/540],...
                'String','0','BackgroundColor',bgcolor);
bl2 = uicontrol('Parent',f,'Style','text','Units','normal','Position',[500/540,54/540,23/540,23/540],...
                'String',num2str(N-1),'BackgroundColor',bgcolor);
bl3 = uicontrol('Parent',f,'Style','text','Units','normal','Position',[170/540,25/540,200/540,23/540],...
                'String','timestep','BackgroundColor',bgcolor);
            
addlistener(b,'ContinuousValueChange',@(hObject, event) updatePlot(hObject, event));


data.b = b;
guidata(f,data);

end

function updatePlot(hObject,event)
    eps = round(hObject.Value);
    data = guidata(hObject);
    data = plotThings(data,eps);
    guidata(hObject,data)
end

function runKF(hObject,event)
    data = guidata(hObject);
    mu = data.mu;
    for i = 1:size(mu,2)
        data = plotThings(data,i);
        data.b.Value = i;
        guidata(hObject,data);
        
        pause(.05)
    end
end

function data = plotThings(data,eps)
    h = data.h;
    mu = data.mu;
    sigma = data.sigma;
    if isgraphics(h)
        delete(h)
        clear h
    end
    h = plotCovEllipse(mu(1:2,eps), sigma(1:2,1:2,eps),1,[{'color'},{'r'},{'linestyle'},{'-'},{'linewidth'},{1}]);
    data.h2.XData = mu(1,eps);
    data.h2.YData = mu(2,eps);
    if size(data.pose,2) > 1
        th = linspace(data.pose(3,eps),2*pi+data.pose(3,eps),25);
        data.hr.XData = [(data.pose(1,eps)+0.13*cos(th)) data.pose(1,eps)];
        data.hr.YData = [(data.pose(2,eps)+0.13*sin(th)) data.pose(2,eps)];
    end
    data.h = h;
    set(data.t,'string',num2str(eps-1))
    drawnow limitrate
end
    