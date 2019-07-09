function [constraints] = myrCostFcn(states,controls) %#codegen
% constraint interface created by falcon.m

% ----------------------------- %
% implement the constraint here %
% ----------------------------- %

% implement constraint values here

% We want balanced flight at the end, with a tangential velocity to the
% centre.
x   = states(1);
y   = states(2);
psi = states(3);

x_th = 50;
y_th = 50;

r = [x-x_th,y-y_th];
v = [sin(psi),cos(psi)];
    
% r should be normal to v.
MCost =  dot(r,v)^2;

constraints = [MCost];
% EoF
end