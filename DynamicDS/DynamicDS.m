% Dynamic simulation

% Assume rate of change of angle of attack is small, so that pitch rate
% determines flightpath curvature and therefore lift.

% Inputs:  pitch rate and roll rate.
% Outputs: velocities about each axis.

% At each timestep, define the forward vector by the current relative
% velocity vector.
% Rotate about this by the integrated roll rate since last timestep.
% Define the acceleration according to the velocity and the pitch rate.
% For a pitch rate, acceleration = v * thetadot.
% And lift coefficient is proportional to acceleration, so thetadot_max is
% proportional to V*CL_max.

close all

addpath('../DS');

clear;

V   = 20; %m/s

DCM = EulerToDCM_ENU(deg2rad(90),0,deg2rad(0));

[roll,pitch,heading] = DCMToEuler_ENU(DCM);

q   = DCMToQuaternion(DCM');
% pos = [0;20;20];
% pos = [0;29;17];
% pos = [0;30;1.5+3];
pos = [0;30;30];
vel = [V;0;0];
dt = 0.01;
g = 0;%9.81;

plotFlag = true;

for iT=1:2000
    
    DCM = QuaternionToDCM(q)';
    
    plotFlag = mod(iT,100)==0;
    [pitch_rate, roll_rate, target_pos] = calculate_guidance(DCM, pos, vel, g, plotFlag);
    
%     pitch_rate = 2/3;
%     roll_rate = 0;

    body_rates = [roll_rate; -pitch_rate; 0];

    q_rates = BodyRatesToQuaternionRates(body_rates, q);
    
    accel = DCM(:,3)*pitch_rate*V + [0;0;-g];
    
    q = q + q_rates*dt;
    
    pos = pos+vel*dt;
    vel = vel+accel*dt;
    
    Pos(iT,:) = pos';
    Target(iT,:) = target_pos';
    Accel(iT,:) = accel';
    ZDir(iT,:) = DCM(:,3)';
    Rates(iT,:) = body_rates';
end

figure,plot3(Pos(:,1),Pos(:,2),Pos(:,3));
hold on;

plot3(Target(:,1), Target(:,2), Target(:,3),'r');
axis equal;