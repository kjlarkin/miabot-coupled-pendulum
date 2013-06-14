% Names: Kevin Larkin and Adam Fisch
% Code to run Miabots following double pendulum trajectory
% June 14, 2013

close all

initial_poses = [.3 0 0 0; -.3 0 0 0]; % initial positions of robots

control_law = @DoublePendulum; % calls control law for robot motion

% calls new Miabot object that actuates robot motion
m = Miabots(initial_poses, control_law, 'velocity', 300, 'sim', true);
m.start

% plots the resulting path of the two robots
figure
plot(m.get_history(1,'state_times'), m.get_history(1,'x'), 'r');
hold on 
plot(m.get_history(2,'state_times'), m.get_history(2,'x'), 'b');