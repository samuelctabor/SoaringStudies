addpath('~/MATLAB/falcon/')

%% Define states, controls and parameters.
x_vec = [...
    falcon.State('x',  0, 100, 0.01);...
    falcon.State('y',  0, 100, 0.01);...
    falcon.State('psi', 0, 50,  0.1)];

u_vec = falcon.Control('Roll', -deg2rad(45), deg2rad(45), 1);


tf = falcon.Parameter('FinalTime', 30.0, 0, 40, 0.1);

%% Define a new optimal control problem.
problem = falcon.Problem('Soaring');


% Specify discretisation.
tau = linspace(0,1,101);

% Add new phase.
phase = problem.addNewPhase(@dynamicsFcn, x_vec, tau, 0, tf);

phase.addNewControlGrid(u_vec, tau);

phase.setInitialBoundaries([0;   0; pi/4]);
% phase.setFinalBoundaries(  [100; 0; pi/2]);

% phase.StateGrid.setSpecificValues(x_vec(1), tau, 1*tau);
% phase.StateGrid.setSpecificValues(x_vec(2), tau, 0*tau);
% phase.StateGrid.setSpecificValues(x_vec(3), tau, 0*tau+pi/2);

% Cost function
 phase.addNewLagrangeCost(@lagrangeCostFcn, falcon.Cost('Cost'));

 % Final cost
 problem.addNewMayerCost(@mayerCostFcn, falcon.Cost('MCost'),phase,1.0);
 
% Add cost function
% problem.addNewParameterCost(tf);

% Solve
problem.Solve();


% Plot
figure
for numState=1:3
    subplot(2,2,numState); grid on; hold on;
    xlabel('time');
    ylabel(phase.StateGrid.DataTypes(numState).Name);

    plot(phase.RealTime, phase.StateGrid.Values(numState,:));
end

figure,subplot(2,2,1);
plot(phase.StateGrid.Values(1,:),phase.StateGrid.Values(2,:));
grid on; grid minor; xlabel('x [m]'); ylabel('y [m]');
axis equal;

subplot(2,2,2);
plot(phase.RealTime,rad2deg(phase.ControlGrids.Values(1,:)));
grid on; grid minor; xlabel('Time [s]'); ylabel('Roll [deg]');

