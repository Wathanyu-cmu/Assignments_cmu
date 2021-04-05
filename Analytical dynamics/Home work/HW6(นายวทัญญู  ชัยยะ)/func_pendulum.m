function [ y ] = func_pendulum( t,x )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global l g
    
    y(1) = x(2);
    y(2) =  -(g*sin(x(1) + 2*x(3)) - 3*g*sin(x(1)) + 2*l*x(2)^2*sin(x(3)) + 2*l*x(4)^2*sin(x(3)) + l*x(2)^2*sin(2*x(3)) + 4*l*x(2)*x(4)*sin(x(3)))/(l*(cos(2*x(3)) - 3));
    y(3) = x(4);
    xdd = y(2);
    y(4) = -(l*sin(x(3))*x(2)^2 + l*xdd + g*sin(x(1) + x(3)) + l*xdd*cos(x(3)))/l ;
    y=y';

end

