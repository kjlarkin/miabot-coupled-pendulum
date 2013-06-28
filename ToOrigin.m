function [ commands ] = ToOrigin(t,states)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
n_robots=size(states, 1);
commands = zeros(n_robots,3);
k1= .02;
k2= .05;
for i=1:n_robots
    x=states(i,1);
    y=states(i,2);
    z=states(i,3);
    v_x=states(i,4);
    v_y=states(i,5);
    theta=states(i,6);
    theta_dot=states(i,7);
    y0=1;
    x0=0;
    
    phi=wrapToPi(atan2(y0-y,x0-x)-theta);
    
    if phi<=pi/2 && phi>-pi/2
        u_theta = k2*sin(phi);
    else
        u_theta = -k2*sin(phi);
    end
    
    r=((x0-x)^2+(y0-y)^2)^.5;
    u_x=k1*r*cos(phi);
    commands(i,1)=u_x;
    commands(i,2)=u_theta;
end
    
end

