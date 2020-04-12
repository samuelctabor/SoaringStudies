close all

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

load('Validation.mat');

n = 2^10;%2^nextpow2(length(validation.t));
ti = linspace(validation.t(1),validation.t(end),n)';

fields = fieldnames(validation);
t = validation.t;
for iF=1:length(fields)
    validation.(fields{iF}) = interp1(t, validation.(fields{iF}), ti,'spline');
%     validation.(fields{iF}) = validation.(fields{iF})(1:end-1);
end
    
x = validation.y';
y = validation.x';
z =-validation.z';
t = validation.t';

gradFunc = @(x,t) interp1((t(1:end-1)+t(2:end))/2, diff(x)./diff(t), t, 'linear','extrap');

vx = gradFunc(x,t);
vy = gradFunc(y,t);
vz = gradFunc(z,t);

phi = -validation.phi';
psi = validation.psi' - pi/2;

V = validation.V;

Uref = 8.560115;
z0 = 0.03;
windFunc = @(x,y,z) [0*x; -Uref*log(z/z0)/log(10/z0); 0*z];


V_rel = windFunc(x,y,z) - [vx;vy;vz];
aspd = norm2(V_rel);

trajectoryPlot(x,y,z,vx,vy,vz,psi,phi,windFunc);

figure;
plot(t,V,'k-x',t,aspd,'k-o',t,norm2([vx;vy;vz]),'b-');
grid on; grid minor;
xlabel('Time [s]'); ylabel('Speed [m/s]');
legend('Validation','Re-calc','Inertial');

figure,plot(t,x,t,y,t,z);


% Fit coefficients.
traj.phi  = atan2(y(end),x(end));
traj.Vnet = sqrt(x(end)^2 + y(end)^2)/(t(end)-t(1));
traj.th   = 0;
traj.clz0 = 1.5;

xLin = traj.Vnet*cos(traj.phi)*cos(traj.th);
yLin = traj.Vnet*sin(traj.phi)*cos(traj.th);

% Back-calculate the required coefficients.
[~,traj.ax,traj.bx] = fitCurve(t, x - xLin*t, N);
[~,traj.ay,traj.by] = fitCurve(t, y - yLin*t, N);
[f,traj.az,traj.bz] = fitCurve(t, z, N);

% Initial guess.
t1 =linspace(0,t(end),1000);
[thrustPower, CL, CD, CT, x1, y1, z1, vx1, vy1, vz1, psi, roll, vectorData] = ...
    dsSoarBackend(t1,windFunc,traj,params,conditions,f);

trajectoryPlot(x1,y1,z1,vx1,vy1,vz1,psi, roll, windFunc, vectorData);

figure,plot(t, validation.phi,t1,roll);
ylabel('Roll');

figure,plot3(x,y,z,'b');
hold on
plot3(x1-x1(1),y1-y1(1),z1,'r');
grid on; grid minor;
axis equal;
title('Trajectories');

figure,plot(t,x-xLin*t,t,y-yLin*t,t,z);
hold on
set(gca,'ColorOrderIndex',1);
plot(t1,x1-x1(1)-xLin*t1,t1,y1-y1(1)-yLin*t1,t1,z1,'LineStyle','--');
grid on; grid minor;
title('Positions');

figure,plot(t, V,       t1, norm2(vectorData.V_rel),...
            t, norm2([vx;vy;vz]), t1, norm2([vx1;vy1;vz1]));
title('Speeds');
legend('Aspd-validation', 'Aspd-Reconstructed',...
       'Inert-validation','Inert-Reconstructed');

figure,plot(t,vx,t,vy,t,vz);
hold on;
set(gca,'ColorOrderIndex',1);
plot(t1,vx1,t1,vy1,t1,vz1,'LineStyle','--');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
title('Velocities');

figure,plot(t1,CL,t,validation.CL,t1,CD,t1,CT);
xlabel('Time [s]');
ylabel('Coef');
legend('Lift','Lift (val)','Drag','Thrust');
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