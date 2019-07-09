function dsTrajectory()

close all;

% % Vehicle parameters.
% params.m = 1;
% 
% params.alpha = -2.5:0.1:20;
% params.CL    = interp1([-2.5,0.0,2.5,5.5,10,15,20],[0.0,0.4,0.84,1.2,1.0,0.8,0.7],params.alpha,'spline','extrap');
% params.CD    = 0.011 + params.CL.^2/(3.14*0.8*24) + (0.0562-0.0227);
% params.S = 1;

params.m = 8.5;
params.alpha = -2.5:0.1:20;
al = [-2.5, 0.0,  2.5, 5.5,  8.2,  12,  13,  20];
cl = [1.8*pi*deg2rad(al(1:6))+0.5,      1.6, 0.7];
cd = 0.033 + (1.8*pi*deg2rad(al)+0.5).^2/(pi*1.0*16.81);
params.CL    = interp1(al,cl, params.alpha,'spline','extrap');
params.CD    = interp1(al,cd, params.alpha,'spline','extrap');
% params.CD    = 0.033 + params.CL.^2/(pi*1.0*16.81);
params.S = 0.65;


% Scenario parameters.
Uref = 8.560115;
z0 = 0.03;

% Wind as function of height.
windFunc = @(x,y,z) [0*x; -Uref*log(z/z0)/log(10/z0); 0*z];

% Use basis functions to construct a trajectory in terms of the differentially
% flat outputs x, y, z.

% Number of basis functions.
N=5;

% Number of times for evaluation.
M = 1000;
t = linspace(0,7,M);

traj_zero.ax = zeros(N,1);
traj_zero.ay = zeros(N,1);
traj_zero.az = zeros(N,1);
traj_zero.bx = zeros(N,1);
traj_zero.by = zeros(N,1);
traj_zero.bz = zeros(N,1);
%
% inclined circle
% traj.ax(1) = 5;
% traj.ay(1) = 5;
% traj.az(1) = 5;
% traj.bx(1) = 0;
% traj.by(1) = pi/2;
% traj.bz(1) = pi;

% Vertical circle
% traj.ax(1) = 5;
% traj.ay(1) = 0;
% traj.az(1) = 5;
% traj.bx(1) = 0;
% traj.by(1) = pi/2;
% traj.bz(1) = pi/2;

% traj.clz0 = 0.5;
% traj.Vnet = 12.1;
% traj.phi  = -0.682;%-pi/4;
% traj.th   = 0.0;

% Validation
traj=load('ValidationTrajectory.mat');

conditions.g    = 9.81;
conditions.rho  = 1.225;

% Initial guess.
[E_val, CL_val, CD_val, CT_val,x_val,y_val,z_val] = ...
    dsSoarBackend(t,windFunc,traj,params,conditions);

% Number of bases that are optimised.
% nb = 2;
nb = N;

traj_init = traj;
traj_init.Vnet = 15.0;

% Set up a function that allows the parameters to be changed.
function [cost, trajMod,Ct]=searchFunc(x)
    trajMod=traj_init;
    trajMod.ax(1:nb) = x(nb*0 + (1:nb));
    trajMod.ay(1:nb) = x(nb*1 + (1:nb));
    trajMod.az(1:nb) = x(nb*2 + (1:nb));
    trajMod.bx(1:nb) = x(nb*3 + (1:nb));
    trajMod.by(1:nb) = x(nb*4 + (1:nb));
    trajMod.bz(1:nb) = x(nb*5 + (1:nb));
    [cost,~,~,Ct]=dsSoarBackend(t,windFunc,trajMod,params,conditions);
end

% Optimise variables in turn for minimum thrust residual.
x0 = [traj_init.ax(1:nb).*(1 + 0.0*randn(nb,1)),...
      traj_init.ay(1:nb).*(1 + 0.0*randn(nb,1)),...
      traj_init.az(1:nb).*(1 + 0.0*randn(nb,1)),....
      traj_init.bx(1:nb).*(1 + 0.0*randn(nb,1)),...
      traj_init.by(1:nb).*(1 + 0.0*randn(nb,1)),...
      traj_init.bz(1:nb).*(1 + 0.0*randn(nb,1))];

[~,trajInit] = searchFunc(x0);
[E_init, CL_init, CD_init, CT_init,x_init,y_init,z_init] = ...
    dsSoarBackend(t,windFunc,trajInit,params,conditions);

if (1)
    options = optimset('Display','iter','MaxFunEvals',10000,'MaxIter',10000);
    x=fminsearch(@searchFunc, x0, options);
    [E_opt,traj]=searchFunc(x);
end

% % xx=traj.ax(1)*linspace(0.8,1.2,20);
% % xs = repmat(x0',1,length(xx));
% % xs(1,:) = xx;
% % Ct = zeros(length(xx),length(t));
% % j = 0*xx;
% % for i=1:size(xs,2)
% %     [j(i),~,Ct(i,:)] = searchFunc(xs(:,i));
% % end
% % figure,plot(xx, j);
% % xlabel('ax(1)');ylabel('\int(T)');


fprintf('Validation cost: %f Initial cost: %f Optimised cost: %f\n',E_val, E_init, E_opt); 

[~, CL, CD, CT, x, y, z, vx, vy, vz, heading, roll, vd] = ...
    dsSoarBackend(t,windFunc,traj,params,conditions);


figure; plot3(x_val,y_val,z_val,x_init,y_init,z_init,x,y,z);
grid on;grid minor;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend('Validation','Initial','Final');


trajectoryPlot(x,y,z,vx,vy,vz,heading, roll, windFunc, vd);

figure,plot(t,CL,t,CD,t,CT);
hold on; set(gca,'ColorOrderIndex',1);
plot(t,CL_val,t,CD_val, t, CT_val,'LineStyle','--')
xlabel('Time [s]');
legend('Lift (opt)','Drag (opt)','Thrust (opt)','Lift (val)','Drag (val)','Thrust (val)')
grid on; grid minor;

figure;
subplot(2,2,1);
plot(t,norm2([vd.acx;vd.acy;vd.acz])/9.81)
xlabel('Time [s]');
grid on; grid minor
ylabel('Acceleration [g]');


subplot(2,2,2);
plot(t,norm2(vd.V_rel),t,norm2([vx;vy;vz]),t,norm2(windFunc(x,y,z)))
xlabel('Time [s]');
legend('Airspeed','Inertial speed','Wind speed');
grid on; grid minor
ylabel('Speed [m/s]');

subplot(2,2,3);
plot(z,norm2(vd.V_rel),z,norm2([vx;vy;vz]),z,norm2(windFunc(x,y,z)))
xlabel('Height [m]');
grid on; grid minor
ylabel('Speed [m/s]');
legend('Airspeed','Inertial speed','Wind speed');

Ek = 0.5*params.m*norm2([vx;vy;vz]).^2;
Ep = params.m*conditions.g*z;

subplot(2,2,4);
plot(t,Ek,t,Ep,t,Ek+Ep);
legend('E_k','E_p','E_t');
xlabel('Time [s]');
grid on; grid minor
ylabel('Energy [J]');

figure;
plot(t,x,t,y,t,z)
hold on;
set(gca,'ColorOrderIndex',1);
plot(t,vx,t,vy,t,vz,'LineStyle','--')
xlabel('Time [s]');
legend('x','y','z')
grid on; grid minor;

V_inertial=[vx;vy;vz];
figure,plot(t,dot(vd.liftFvec, V_inertial),...
            t,dot(0*vd.liftFvec+[0;0;-params.m*conditions.g], V_inertial),...
            t,dot(vd.dragFvec, V_inertial));
xlabel('Time [s]');
ylabel('Force [N]');
legend('Lift','Gravity','Drag');
grid on; grid minor;

end
   
