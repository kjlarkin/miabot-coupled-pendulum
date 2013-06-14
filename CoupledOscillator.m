classdef CoupledOscillator < handle
    %   CoupledOscillator uses two robots to simulate the motion of 
    %   two pendulums which are attatched by a spring.  Spring constant 
    %   (k), pendulum length (l), mass (m), and initial conditions t1,
    %   t1_dot, t2, t2_dot, are adjustable.
    %
    %   METHODS
    %   
    
    properties
        k1 = 2;           % control constant for velocity
        k2 = 1;           % control constant for heading angle
        ks = 1;           % spring coefficient
        l = 5;            % rope length
        m = 20;           % pendulum mass
        g = 9.8;          % gravity constant
        Offset = .3;      % displacement from the origin to equilibrium
        theta1 = 0;       % displacement theta for pendulum 1
        theta1_dot = .3;  % initial angular velocity for pendulum 1
        theta2 = 0;       % displacement theta for pendulum 2
        theta2_dot = .1;  % initial angular velocity for pendulum 2
        
        initial_poses;    % the initial locations of the pendula (4x2)
    end
    properties (Hidden)
        runTime;          % time to run the robot for
        lambda;           % period coeff. for mode 1
        omega;            % period coeff. for mode 2
        
        % coefficients for the pendulum equations of motion
        c1;
        c2;
        c3;
        c4;
    end
    
    methods
        function obj = CoupledOscillator(initial_poses)   
            % initializes a new Coupled Oscillator class with a 2x4 matrix
            
            obj.initial_poses = initial_poses;     
        end
        
        
        function [ commands ] = DoublePendulum(obj, t, states)
            
            n_robots = size(states,1);  % number of robots (this will be 2)
            commands = zeros(n_robots,3); % init. command matrix as 0's
            
            % loop to set positions for robots
            for i=1:n_robots
                
                % Get current states of the robot, x,y,z,heading, and
                % velocities
                x = states(i,1);
                y = states(i,2);
                z = states(i,3);
                v_x = states(i,4);
                v_y = states(i,5);
                theta = states(i,6);
                theta_dot = states(i,7);
                
                % set desired y position (always 0)
                ygoal = 0;
                
                % If statement handles both theta functions for pendulums
                % 1 & 2
                if i == 1
                    xgoal = obj.Pendulum1(t); 
                else
                    xgoal = obj.Pendulum2(t);
                end
                
                % angle that the current heading is displaced from desired
                % heading
                phi = wrapToPi(atan2(ygoal-y,xgoal-x)-theta);
                
                % if statement to determine control laws for angular
                % velocity
                if phi <= pi/2 && phi > -pi/2
                    u_theta = (obj.k2)*sin(phi);
                else
                    u_theta = -(obj.k2)*sin(phi);
                end
                
                r = ((xgoal-x)^2+(ygoal-y)^2)^.5; % distance to goal
                                                  % position
                
                u_x = (obj.k1)*r*cos(phi); % control law for forward
                                           % velocity
                
                % pass forward velocity and angular velocity to the command
                % matrix
                commands(i,1) = u_x;
                commands(i,2) = u_theta;
            end
            
        end
        
        function [] = start(obj, runTime)
            close all
            obj.runTime = runTime;
            
            obj.lambda = sqrt(2*((obj.ks)/(obj.m)) + (obj.g)/(obj.l)); 
            obj.omega = sqrt((obj.g)/(obj.l));         
            
            obj.c1 = (obj.theta1_dot - obj.theta2_dot)/(2*(obj.omega));
            obj.c2 = (obj.theta1 - obj.theta2) / (2);
            obj.c3 = (obj.theta1_dot + obj.theta2_dot)/(2*(obj.lambda));
            obj.c4 = (obj.theta1 + obj.theta2) / (2);
            
            % call control law for robot motion
            control_law = @(t,x) obj.DoublePendulum(t,x); 
            
            
            % calls new Miabot object that actuates robot motion
            m = Miabots(obj.initial_poses, control_law, 'velocity',...
                obj.runTime, 'sim', true);
            m.start
            u=0:.1:obj.runTime;
            
            % plots the resulting path of the two robots against the ideal
            figure
            plot(m.get_history(1,'state_times'), m.get_history(1,'x'),...
                 m.get_history(2,'state_times'), m.get_history(2,'x'),...
                 u, (obj.Pendulum1(u)), u, (obj.Pendulum2(u)));
            
        end
        
        function [ xgoal ] = Pendulum1(obj, t)
            % Determines the path of the first of two linked pendulums
            % using the equation of motion
            
            xgoal = (obj.c1)*sin((obj.omega)*t) + ...
            (obj.c2)*cos((obj.omega)*t) + (obj.c3)*sin((obj.lambda)*t)...
            + (obj.c4)*cos((obj.lambda)*t) + obj.Offset;
            
        end
        
        function [ xgoal ] = Pendulum2(obj, t)
            % Determines the path of the second of two linked pendulums
            % using the equation of motion
            
            xgoal = -((obj.c1)*sin((obj.omega)*t) + ...
            (obj.c2)*cos((obj.omega)*t) - (obj.c3)*sin((obj.lambda)*t)...
                - (obj.c4)*cos((obj.lambda)*t) + obj.Offset);
            
        end
        
    end
end

