close all
clear;

params.m = 8.5;
params.alpha = -2.5:0.1:20;
al = [-2.5, 0.0,  2.5, 5.5,  8.2,  12,  13,  20];
cl = [1.8*pi*deg2rad(al(1:6))+0.5,      1.6, 0.7];

params.CL    = interp1(al,cl, params.alpha,'spline','extrap');

params.CD    = 0.033 + params.CL.^2/(pi*1.0*16.81);
params.S = 0.65;

figure,plot(al,cl,'.:',params.alpha,params.CL)

conditions.g   = 9.81;
conditions.rho = 1.225;

N = 32;
% N = 8;

Uref = 8.560115;
z0 = 0.03;
windFunc = @(x,y,z) [0*x; -Uref*log(z/z0)/log(10/z0); 0*z];


traj.Vnet = 0;
traj.phi  = 0;
traj.clz0 = 1.5;
traj.th = 0;

traj.ax = 30;
inc = 15; %deg
traj.ay = cosd(inc)*30;
traj.az = sind(inc)*30;
traj.bx = 0;
traj.by = pi/2;
traj.bz = pi/2;

T = 6;
f = 1/T;

% Initial guess.
t1 =linspace(0,T,1000);
[thrustPower, CL, CD, CT, x1, y1, z1, vx1, vy1, vz1, psi, roll, vectorData] = ...
    dsSoarBackend(t1,windFunc,traj,params,conditions,f);

trajectoryPlot(x1,y1,z1,vx1,vy1,vz1,psi, roll, windFunc, vectorData);

figure,
plot3(x1-x1(1),y1-y1(1),z1,'r');
grid on; grid minor;
axis equal;
title('Trajectories');

figure,plot(t1, norm2(vectorData.V_rel),...
            t1, norm2([vx1;vy1;vz1]));
title('Speeds');
legend('Aspd-Reconstructed',...
       'Inert-Reconstructed');

figure,plot(t1,vx1,t1,vy1,t1,vz1,'LineStyle','--');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Velocities');

figure,plot(t1,CL,t1,CD,t1,CT);
xlabel('Time [s]');
ylabel('Coef');
legend('Lift','Drag','Thrust');
grid on; grid minor;
title('Coefficients');


Ek = 0.5*params.m*norm2([vx1;vy1;vz1]).^2;
Ek_air = 0.5*params.m*norm2(vectorData.V_rel).^2;
Ep = params.m*conditions.g*z1;

figure,plot(t1,Ek,t1,Ep,t1,Ek+Ep,t1,Ek_air,t1,Ep+Ek_air);
legend('E_k','E_p','E_t','E_{k,air}','E_{t,air}');
xlabel('Time [s]');
grid on; grid minor
ylabel('Energy [J]');