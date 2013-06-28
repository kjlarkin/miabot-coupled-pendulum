classdef CoupledOscillator < handle
    %{ 
       Names: Kevin Larkin and Adam Fisch
       Class to run Miabot robots according to a coupled oscillator path
       June 17, 2013
    
       CoupledOscillator uses two robots to simulate the motion of two
       pendulums which are attatched by a spring.  Spring constant 
       (k), pendulum length (l), mass (m), and initial conditions 
       theta1_initial, theta1_dot_initial, theta2_initial, 
       theta2_dot_initial, are adjustable.
    
       METHODS
       
       start - begins a coupled oscillator movement command sequence and
       displays the desired and actual results.
    
       DoublePendulum - takes in current states and time, and tells the
       robots where to travel, based on the control law and pendulum
       equations.
    
       Pendulum1 - takes in the current time, and determines where the
       first pendulum should be.
    
       Pendulum2 - takes in the current time, and determines where the
       second pendulum should be.
    
       DEPENDENCIES
       
       Miabots.m
     
    %}
    
    
    properties
        k1 = 2;           % control constant for velocity
        k2 = 1;           % control constant for heading angle
        k3 = .1;          % control constant for velocity integral control
        ks = 1;           % spring coefficient
        l = 5;            % pendulum rope length
        m = 20;           % pendulum mass
        g = 9.8;          % gravity constant
        Offset = .3;      % displacement from the origin to equilibrium
        theta1_initial = 0;        % displacement theta for pendulum 1
        theta1_dot_initial = .3;   % initial angular velocity for pendulum 1
        theta2_initial = 0;        % displacement theta for pendulum 2
        theta2_dot_initial = -.1;  % initial angular velocity for pendulum 2
        
        initial_poses;    % the initial locations of the robots (4x2)
        sum_error_x = [0; 0];
        runTime;          % time to run the robot for
    end
    properties (Access = private)
        
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
        
        
        function [ commands ] = ControlLaw(obj, t, states)
            
            n_robots = size(states,1);  % number of robots (this will be 2)
            commands = zeros(n_robots,3); % init. command matrix as 0's
            
            obj.setConstants();
            
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
                                                  
                if i == 1
                    dxgoal = obj.dPendulum1(t);
                else
                    dxgoal = obj.dPendulum2(t);
                end
                
                devMag = dxgoal / ((dxgoal)^2)^.5;
               
                obj.sum_error_x(i) = obj.sum_error_x(i) + (((xgoal - x)) *.04 * devMag);
                
                % control law for forward velocity
                u_x = ((obj.k1)*r + ...
                (obj.k3)*(obj.sum_error_x(i)))*cos(phi); 
                
                % pass forward velocity and angular velocity to the command
                % matrix
                commands(i,1) = u_x;
                commands(i,2) = u_theta;
            end
            
        end
        
        function [] = setConstants(obj)
            % calculate the coefficients for equations of motion based on
            % initial variables
            
            obj.lambda = sqrt(2*((obj.ks)/(obj.m)) + (obj.g)/(obj.l)); 
            obj.omega = sqrt((obj.g)/(obj.l));         
            
            obj.c1 = (obj.theta1_dot_initial - obj.theta2_dot_initial) /...
                (2*(obj.omega));
            obj.c2 = (obj.theta1_initial - obj.theta2_initial) / (2);
            obj.c3 = (obj.theta1_dot_initial + obj.theta2_dot_initial) /...
                (2*(obj.lambda));
            obj.c4 = (obj.theta1_initial + obj.theta2_initial) / (2);
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
        
        function [ dxgoal ] = dPendulum1(obj, t)
            % Determines the derivative of the velocity of the first of two
            % linked pendulums using the equation of motion
            
            dxgoal = (obj.c1 * obj.omega)*cos((obj.omega)*t) - ...
            (obj.c2 * obj.omega)*sin((obj.omega)*t) + ...
            (obj.c3 * obj.lambda)*cos((obj.lambda)*t) - ...
            (obj.c4 * obj.lambda)*sin((obj.lambda)*t);
        end
        
        function [ dxgoal ] = dPendulum2(obj, t)
            % Determines the derivative of the velocity of the second of
            % two linked pendulums using the equation of motion
            
            dxgoal = -(obj.omega * obj.c1)*cos((obj.omega)*t) + ...
            (obj.c2 * obj.omega)*sin((obj.omega)*t) + ...
            (obj.c3 * obj.lambda)*cos((obj.lambda)*t) - ...
            (obj.c4 * obj.lambda)*sin((obj.lambda)*t);
            
        end
        
    end
end

