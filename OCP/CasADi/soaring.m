close all

% Thermal updraft optimal capture
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP
addpath('~/MATLAB/CasADi/casadi-linux-matlabR2014b-v3.4.4/');
N = 100; % number of control intervals

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(4,N+1); % state trajectory

U = opti.variable(1,N+1);   % control trajectory (roll rate)
T = 30.0;
V = 10.0;
W = 5.0;
R = 30.0;
xth = [50;50];
g = 9.81;

maxU = deg2rad(10); % rad/s
maxRoll = deg2rad(45); % rad

j_func = @(X,U) 2.0*X(4,:).^2 + ...
                20.0*U.^2 + ...
               -W*exp(-((X(1,:)-xth(1)).^2+(X(2,:)-xth(2)).^2)/R^2);
             
opti.minimize(sum(j_func(X,U)));
          
% ---- dynamic constraints --------
state_vector = {'x';
               'y';
               'psi'; % (heading)
               'phi'};% (roll angle)]
% dx/dt = f(x,u)
f = @(x,u) [sin(x(3))*V;...
            cos(x(3))*V;...
            g*sin(x(4))/V;...
            u]; 

dt = T/N; % length of a control interval
for k=1:N % loop over control intervals
   % Runge-Kutta 4 integration
   k1 = f(X(:,k),         U(:,k));
   k2 = f(X(:,k)+dt/2*k1, U(:,k));
   k3 = f(X(:,k)+dt/2*k2, U(:,k));
   k4 = f(X(:,k)+dt*k3,   U(:,k));
   x_next = X(:,k) + dt/6*(k1+2*k2+2*k3+k4); 
   opti.subject_to(X(:,k+1)==x_next); % close the gaps
end

% ---- path constraints -----------
opti.subject_to(-maxU<=U<=maxU);           % control is limited
opti.subject_to(-maxRoll<=X(4,:)<=maxRoll);           % roll is limited

% ---- boundary conditions --------
init_phi = pi/4;
opti.subject_to(X(1,1)==0);   % start at position 0 ...
opti.subject_to(X(2,1)==0);   % start at position 0 ...
opti.subject_to(X(3,1)==init_phi);   % start at position 0 ...
opti.subject_to(X(4,1)==0);   % start at position 0 ...

endRelPos = [X(1,end)-xth(1), X(2,end)-xth(2)];
endVector = [sin(X(3,end)), cos(X(3,end))];
opti.subject_to(dot(endRelPos, endVector)/sqrt(sum(endRelPos.^2))==0); % final heading is aligned

reqROT  = V/sqrt(sum(endRelPos.^2));
reqRoll = asin(V*reqROT/g);
% opti.subject_to(-X(4,end)==reqRoll); % balanced turn

%----- initial guess ==========

% opti.set_initial(X(1,:), [linspace(0,sin(init_phi)*V*T, N+1)]);
% opti.set_initial(X(2,:), [linspace(0,cos(init_phi)*V*T, N+1)]);
% opti.set_initial(X(3,:), repmat(init_phi, N+1, 1));

% ---- solve NLP              ------
opti.solver('ipopt'); % set numerical backend
sol = opti.solve();   % actual solve

% ---- post-processing        ------

figure
hold on
j = sol.value(j_func(X,U));

plot(sol.value(X(1,:)),sol.value(X(2,:)),'Color',[0.8,0.8,0.8]);
scatter(sol.value(X(1,:)),sol.value(X(2,:)), 10, j, 'filled');
grid on; grid minor;
title('Trajectory')
axis equal;
xlabel('x [m]');
ylabel('y [m]');
colorbar;

%
% Plot a bunch of stuff.
%
figure
hold on

% Extract solution state trajectory.
Xv = sol.value(X);

% Wrap psi (heading) to -pi->pi
Xv(3,:) = mod(Xv(3,:),2*pi);
Xv(3,(Xv(3,:)>pi)) = Xv(3,(Xv(3,:)>pi)) - 2*pi;

% Wrap phi (roll) to -pi->pi
Xv(4,(Xv(4,:)>pi)) = Xv(4,(Xv(4,:)>pi)) - 2*pi;

% Plot as degrees
Xv(3,:) = rad2deg(Xv(3,:));
Xv(4,:) = rad2deg(Xv(4,:));

% Plot state vector.
plot(1:(N+1),Xv);

% Plot control inputs.
stairs(1:(N+1),rad2deg(sol.value(U)),'k');

% Plot the maximum roll and input constraints.
plot(1:(N+1),rad2deg(maxRoll *repmat([1;-1],1,N+1)),'r--');
plot(1:(N+1),rad2deg(maxU    *repmat([1;-1],1,N+1)),'b--');
legend([state_vector;{'U'}])
