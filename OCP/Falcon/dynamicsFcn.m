function [states_dot] = dynamicsFcn(states, controls) %#codegen
% model interface created by falcon.m

% Extract states
x    = states(1);
y    = states(2);
psi  = states(3);

% Extract controls
Roll = controls(1);

% ------------------------ %
% implement the model here %
% ------------------------ %

% implement state derivatives here
V = 10;
g = 9.81;

x_dot   = V*sin(psi);
y_dot   = V*cos(psi);
psi_dot = g*sin(Roll)/V;

states_dot = [x_dot; y_dot; psi_dot];
% EoF
end