function [costs] = lagrangeCostFcn(states, controls)
% cost function interface created by falcon.m

% Extract states
x    = states(1);
y    = states(2);
psi  = states(3);

% Extract controls
Roll = controls(1);

% ----------------------------- %
% implement the cost function here %
% ----------------------------- %
x_th = 50;
y_th = 50;

% implement cost values here
r2 = (x-x_th).^2 + (y-y_th).^2;
Cost = - exp(-r2/50^2) + 0.05*Roll.^2;
costs = [Cost];
% EoF

end