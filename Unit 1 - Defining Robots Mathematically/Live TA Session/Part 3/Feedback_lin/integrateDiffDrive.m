function finalpose = integrateDiffDrive(initialpose, v, omega, dt)
% integrateDiffDrive: integrate the robot odometry to calculate its pose after
% it traveled a distance d and turned an angle phi
% 
%   FINALPOSE = integrateDiffDrive(INITIALPOSE,D,PHI) returns the robot final
%   pose after it traveles a distance D and turned an angle PHI
% 
%   INPUTS
%       initialpose   1-by-3 pose vector in global coordinates (x,y,theta)
%       v             N-by-1 vector of commanded velocities
%       phi           N-by-1 vector of commanded angular velocities
%       dt            N-by-1 vector of timesteps (single value is fine)
% 
%   OUTPUTS
%       finalpose     (N+1)-by-(3) pose vector in global coordinates (x,y,theta)
% 
% 
%   eCornell
%   MAE 101: Autonomous Mobile Robots
%   Homework 3
%   Neveln


n = length(v);
x = initialpose(1);
y = initialpose(2);
theta = initialpose(3);

if length(dt) == 1
    dt = dt.*ones(size(v));
end

x_f = zeros(n+1,1);
y_f = zeros(n+1,1);
theta_f = zeros(n+1,1);
finalpose = zeros(n+1,3);


x_f(1) = x;
y_f(1) = y;
theta_f(1) = theta;

for i=1:n
    if omega(i) == 0
        dx = v(i)*dt(i)*cos(theta_f(i));
        dy = v(i)*dt(i)*sin(theta_f(i));
        dtheta = 0 ;
    elseif v == 0   % Turning only
        dx = 0;
        dy = 0;
        dtheta = omega(i)*dt(i);
    else
        sign_v= sign(v(i));
        dtheta= omega(i)*dt(i);
        dir= sign(dtheta);
        motionRad = v(i)/omega(i);
        l_chord= motionRad*2*sin(dtheta/2);
        y_rel= -dir*l_chord^2/2/motionRad;
        x_rel= sign_v*sqrt(l_chord^2-y_rel^2);
        %translate into global chords 
        R= [cos(dtheta + theta_f(i)) -sin(dtheta + theta_f(i)) ; sin(dtheta + theta_f(i)) cos(dtheta + theta_f(i))];
        dxdy=  R*[x_rel ; y_rel];
        dx = dxdy(1);
        dy = dxdy(2);
%             sum_x = sum_x + 2*(d(i)/phi(i))*sin(phi(i)/2)*cos(theta_f(i) + phi(i)/2);
%             sum_y = sum_y + 2*(d(i)/phi(i))*sin(phi(i)/2)*sin(theta_f(i) + phi(i)/2);
%         sum_theta = sum_theta + phi(i);
    end
    x_f(i+1) = dx + x_f(i);
    y_f(i+1) = dy + y_f(i);
    theta_f(i+1) = dtheta + theta_f(i);
end

finalpose(:,1) = x_f;
finalpose(:,2) = y_f;
finalpose(:,3) = theta_f;
end