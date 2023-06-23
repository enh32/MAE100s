function [ dydt ] = diff_drive( t,y,v,w )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

dydt = zeros(3,1);
dydt(1) = v*cos(y(3));
dydt(2) = v*sin(y(3));
dydt(3) = w;

end

