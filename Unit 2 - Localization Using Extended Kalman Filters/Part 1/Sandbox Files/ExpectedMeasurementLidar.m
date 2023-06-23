function[measurements] = ExpectedMeasurementLidar(robotPose, alpha, map, maxRange,varargin)
% ExpectedMeasurement: predict the range measurements for a point robot 
% with sensors at angles alpha given a map and the maxRange of the sensors
% 
%   measurements = ExpectedMeasurement(ROBOTPOSE,ALPHA,MAP,MAXRANGE) returns 
%   the expected range measurements for a point robot operating in a known 
%   map. 
% 
%   INPUTS
%       robotPose   3-by-1 pose vector in global coordinates (x,y,th)
%       alpha       M-by-1 vector of range sensor angles
%       map         N-by-4 matrix containing the coordinates of walls in 
%                   the environment: [x1, y1, x2, y2]
%       maxRange    maximum sonar range (meters)
% 
%   OUTPUTS
%       measurements       M-by-1 vector of ranges (meters)
% 
% 
%   Cornell University

% initializing variables
if nargin > 4
    robotRad = varargin{1};
else   
    robotRad = 0.13;
end

if length(robotPose) == 3
    th_sensor = robotPose(3,1);
else
    th_sensor = 0;
end
x_sensor = robotPose(1,1)+robotRad*cos(th_sensor);
y_sensor = robotPose(2,1)+robotRad*sin(th_sensor);

measurements = zeros(length(alpha),1);
for k = 1:length(alpha)
    th = alpha(k)+th_sensor;
    % Create line of sight
    x_range= x_sensor+maxRange*cos(th);   % Range of sensor
    y_range= y_sensor+maxRange*sin(th);

    % Find line equation for sensor line
    m_sensor= (y_range-y_sensor)/(x_range-x_sensor);
    if m_sensor > 1e14
        m_sensor= inf;
    elseif abs(m_sensor) < 1e-14
        m_sensor= 0;
    elseif m_sensor < -1e14
        m_sensor= -inf;
    end
    b_sensor= y_sensor-m_sensor*x_sensor;

    % Check against every obstacle (individually)
    j= 1;                   % Count variable for intersections
    x_int= [];              % Position of intersections
    y_int= [];
    for i= 1:size(map,1)% Count variable for obstacles
        % Find line equations for wall lines
        m_wall= (map(i,4)-map(i,2))/...
            (map(i,3)-map(i,1));
        if m_wall > 1e14
            m_wall= inf;
        elseif abs(m_wall) < 1e-14
            m_wall= 0;
        elseif m_wall < -1e14
            m_wall= -inf;
        end
        b_wall= map(i,2)-m_wall*map(i,1);

        % Find intersection of infinitely long walls
        if ~(m_sensor == m_wall)    % Not parallel lines
            if isinf(m_sensor)      % Vertical sensor line
                x_hit= x_sensor;
                y_hit= m_wall*x_hit+b_wall;
            elseif isinf(m_wall)    % Vertical wall line
                x_hit= map(i,1);
                y_hit= m_sensor*x_hit+b_sensor;
            else                    % Normal conditions
                x_hit= (b_wall-b_sensor)/(m_sensor-m_wall);
                y_hit= m_sensor*x_hit+b_sensor;
            end

            % Verify that intersection is on finite lines
            % Use tolerances to account for rounding errors on
            % vertical or horizontal lines
            if x_hit-min(x_sensor,x_range) > -0.001 && ...
                    x_hit-max(x_sensor,x_range) < 0.001 && ...
                    y_hit-min(y_sensor,y_range) > -0.001 && ...
                    y_hit-max(y_sensor,y_range) < 0.001 && ...
                    x_hit-min(map(i,[1 3])) > -0.001 && ...
                    x_hit-max(map(i,[1 3])) < 0.001 && ...
                    y_hit-min(map(i,[2 4])) > -0.001 && ...
                    y_hit-max(map(i,[2 4])) < 0.001
                x_int(j)= x_hit;
                y_int(j)= y_hit;
                j= j+1;
            end
        end
    end

    % Find closest wall on sensor line
    dist= maxRange;    % Initialize to max range
    if ~isempty(x_int)
        distVec= sqrt((x_int-x_sensor).^2+(y_int-y_sensor).^2);
        dist= min(distVec);  % Find shortest distance to intersections
    end
    measurements(k) = dist;
end


end

