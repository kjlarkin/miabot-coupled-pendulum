
% Names: Kevin Larkin and Adam Fisch
% Control law code for robots following double pendulum trajectory
% June 14, 2013

function [ commands ] = DoublePendulum(t,states)
%   DoublePendulum uses two robots to simulate the motion of two pendulums
%   which are attatched by a spring.  Spring constant (k), pendulum length
%   (l), mass (m), and initial conditions t1, t1_dot, t2, t2_dot, are
%   adjustable.

n_robots = size(states, 1);    % number of robots (this will be 2)
commands = zeros(n_robots,3);  % initialize command matrix as 0's

% Constants
k1= 2;        % control constant for velocity                     
k2= 1;        % control constant for heading angle
k = 1;        % spring coefficient
l = 5;        % rope length
m = 20;       % pendulum mass
g = 9.8;      % gravity constant
OrgDisp = .3; % displacement from the origin

% initial conditions for both pendulums
t1 = 0;                    % displacement theta for pendulum 1
t1_dot = .3;                 % initial angular velocity for pendulum 1 
t2 = 0;                    % displacement theta for pendulum 2
t2_dot = .1;                 % initial anglular velocity for pendulum 2
lambda = sqrt(2*k/m + g/l); % period coeff. for mode 1
omega = sqrt(g/l);          % period coeff. for mode 2

% constants for the sol'n to the double pendulum eqn.
c1 = (t1_dot - t2_dot)/(2*omega);
c2 = (t1 - t2) / (2);
c3 = (t1_dot + t2_dot)/(2*lambda);
c4 = (t1 + t2) / (2);

% loop to set positions for robots
for i=1:n_robots

    % Get current states of the robot, x,y,z,heading, and velocities
    x = states(i,1);   
    y = states(i,2);   
    z = states(i,3);
    v_x = states(i,4);
    v_y = states(i,5);
    theta = states(i,6);
    theta_dot = states(i,7);
    
    % set desired y position (always 0)
    y0 = 0;
    
    % If statement handles both theta functions for pendulums 1 & 2
    if i == 1
        x0 = c1*sin(omega*t) + c2*cos(omega*t) ...
             + c3*sin(lambda*t) + c4*cos(lambda*t) + OrgDisp;
    else
        x0 = -(c1*sin(omega*t) + c2*cos(omega*t) ...
             - c3*sin(lambda*t) - c4*cos(lambda*t) + OrgDisp);
    end
    
    % angle that the current heading is displaced from desired heading
    phi = wrapToPi(atan2(y0-y,x0-x)-theta); 
    
    % if statement to determine control laws for angular velocity
    if phi <= pi/2 && phi > -pi/2
        u_theta = k2*sin(phi);
    else
        u_theta = -k2*sin(phi);
    end
    
    r=((x0-x)^2+(y0-y)^2)^.5; % distance to goal position
    
    u_x=k1*r*cos(phi); % control law for forward velocity
    
    % pass forward velocity and angular velocity to the command matrix
    commands(i,1)=u_x;
    commands(i,2)=u_theta;
end
    
end

