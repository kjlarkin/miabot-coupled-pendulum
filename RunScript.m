%clear all
c = CoupledOscillator([.3 0 0 0; -.3 0 0 0]);
close all
c.runTime = 300;
            
% call control law for robot motion
control_law = @(t,x) c.ControlLaw(t,x); 
            
% calls new Miabot object that actuates robot motion
m = Miabots(c.initial_poses, control_law, 'velocity', c.runTime,...
    'sim', true);
m.start
u=0:.04:c.runTime;
            
% plots the resulting path of the two robots against the ideal
figure
plot(m.get_history(1,'state_times'), m.get_history(1,'x'), 'r',...
m.get_history(2,'state_times'), m.get_history(2,'x'), 'g', u,...
(c.Pendulum1(u)), 'b', u, (c.Pendulum2(u)), 'b');
            
xlabel('Time (s)');
ylabel('X-position (m)');
legend('Robot 1', 'Robot 2', 'Goal Trajectories');
title('Trajectory of Miabots Simulating a Coupled Pendulum');
            