close all

% Thermal updraft optimal capture
% ----------------------
% An optimal control problem (OCP),
% solved with direct multiple-shooting.
%
% For more information see: http://labs.casadi.org/OCP
addpath('~/Documents/MATLAB/CasADi/casadi-linux-matlabR2014b-v3.5.1/');
N = 50; % number of control intervals

integration_type = 'BackwardsEuler';

opti = casadi.Opti(); % Optimization problem

% ---- decision variables ---------
X = opti.variable(4,N+1); % state trajectory
U = opti.variable(1,N+1);   % control trajectory (roll rate)


T = 20.0;
V = 10.0;
W = 5.0;
R = 30.0;
xth        = [50;50];
g          = 9.81;
numXInit   = [0;0];
numPhiInit = pi/4;

% Best thermalling radius.
r   = 0.3:0.1:2.0 * R;
roll = atan(V^2./(g*r));

j = 2*roll.^2 - W*exp(-r.^2/R^2);
[~, idx] = min(j);
r_opt = 20;%r(idx);

% Initial solution based on L1 control.
init_sol = compute_initial_solution(numXInit, numPhiInit, xth, V, r_opt, N+1, T);
init_sol.U = init_sol.U';
init_sol.X = init_sol.X';


numMaxU    = deg2rad(8); % rad/s
numMaxRoll = deg2rad(40); % rad

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

% Symbolic function for x_next.
xpos = casadi.MX.sym('x'); % States
ypos = casadi.MX.sym('y');
psi  = casadi.MX.sym('psi');
phi  = casadi.MX.sym('phi');

x  = [xpos; ypos; psi; phi];
u  = casadi.MX.sym('u'); % Controls
x_n = casadi.MX.sym('x_n',4);

switch integration_type
    case 'RK4'
        % Runge-Kutta 4 integration
        k1 = f(x,         u);
        k2 = f(x+dt/2*k1, u);
        k3 = f(x+dt/2*k2, u);
        k4 = f(x+dt*k3,   u);
        x_next = x + dt/6*(k1+2*k2+2*k3+k4);
    case 'ForwardsEuler'
        x_next = x + dt*f(x,u);
    case 'BackwardsEuler'
        % In this case use the state at end of interval to evaluate
        % gradient.
        x_next = x + dt*f(x_n, u);
end

F = casadi.Function('F',{x,x_n,u},{x_next},{'x','x_next','u'},{'x_next'});

for k=1:N % loop over control intervals
    opti.subject_to(X(:,k+1) == F(X(:,k),X(:,k+1),U(:,k)));
end

maxU     = opti.parameter;
maxRoll  = opti.parameter;

xInit    = opti.parameter(2,1);
phiInit  = opti.parameter;


% ---- path constraints -----------
opti.subject_to(-maxU   <= U    <=maxU);           % control is limited
opti.subject_to(-maxRoll<=X(4,:)<=maxRoll);           % roll is limited

% ---- boundary conditions --------

% -- Start--
opti.subject_to(X(1,1)==xInit(1));
opti.subject_to(X(2,1)==xInit(2));
opti.subject_to(X(3,1)==phiInit);
opti.subject_to(X(4,1)==0);


% -- End--
endRelPos = [X(1,end)-xth(1), X(2,end)-xth(2)];
endRadius = sqrt(sum(endRelPos.^2));
endVector = [sin(X(3,end)), cos(X(3,end))];

opti.subject_to(dot(endRelPos, endVector)/sqrt(sum(endRelPos.^2))==0); % final heading is tangent

reqRoll = sign(init_sol.X(4,end))*asin(V^2/(r_opt*g));
opti.subject_to((X(4,end))==reqRoll); % balanced turn

% tngential velocity
opti.subject_to((endRelPos(1)*endVector(2) - endRelPos(2)*endVector(1)) == -sign(init_sol.X(4,end))*endRadius);
opti.subject_to(endRadius == r_opt); % final radius

% ---- solver options    ------
if (1)
    opti.solver('ipopt'); % set numerical backend
else
    % Make the solver silent
    opts = struct;
    opts.qpsol = 'qrqp';
    opts.print_header = false;
    opts.print_iteration = false;
    opts.print_time = false;
    opts.qpsol_options.print_iter = false;
    opts.qpsol_options.print_header = false;
    opts.qpsol_options.print_info = false;
    opti.solver('sqpmethod',opts);
end

% Create a function for later use
M = opti.to_function('M',{maxRoll,  maxU,  xInit,  phiInit,   U,     X},      {U(:,1)},...
                        {'maxRoll','maxU','xInit','phiInit', 'Uinit','Xinit'},{'u_opt'});

M.save('OptimalSoaringFunc');

save('InitialSolution.mat','-struct', 'init_sol');

M2=casadi.Function.load('OptimalSoaringFunc');
tic
ret = M2(numMaxRoll,numMaxU, numXInit, numPhiInit, init_sol.U, init_sol.X)
toc


% Do the normal solution
opti.set_value(maxU,    numMaxU);
opti.set_value(maxRoll, numMaxRoll);
opti.set_value(xInit, numXInit);
opti.set_value(phiInit, numPhiInit);

opti.set_initial(X, init_sol.X);
opti.set_initial(U, init_sol.U);

tic
sol = opti.solve();
sol.value(U(1))
toc


% ---- post-processing        ------

figure,plot(init_sol.X(2,:), init_sol.X(1,:),'b--');

th=0:0.1:2*pi;
hold on;
plot(xth(1)+r_opt*sin(th), xth(2)+r_opt*cos(th),'r--');

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
plot(1:(N+1),rad2deg(numMaxRoll *repmat([1;-1],1,N+1)),'r--');
plot(1:(N+1),rad2deg(numMaxU    *repmat([1;-1],1,N+1)),'b--');
legend([state_vector;{'U'}])

% Plot initial vs final
figure,plot(1:N+1,init_sol.X)
set(gca,'ColorOrderIndex',1)
hold on
plot(1:N+1, sol.value(X),'--')
grid on; grid minor
legend('X','Y','Hdg','Roll');