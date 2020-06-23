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
g   = 9.81;
Vw  = 5; %m/s at 10m

% Some assumed figures.
CLK = 1.0*2*pi/8.0; % S*CLalpha/mass
CDK = 1.0*0.02/8.0; % S*D/mass

qdyn = 0.5*1.225*V^2;
dGdAlpha = qdyn*CLK;

% Initial conditions
% 20 deg inclination, small error
if (1)
    trajSpec.inclination = 20;
    trajSpec.R = 30;
    pos = [0;30;30];
    vel = [V;0;0];
    roll    = 90;
    pitch   = 0;
    heading = 0;
    accelreq = sqrt((V^2/trajSpec.R)^2 + 9.81^2);
    alpha_trim   = accelreq/(qdyn*CLK);
else
    % Flat turn.
    trajSpec.inclination = 0;
    trajSpec.R = 30;
    pos = [0; trajSpec.R; 1.5];
    vel = [V;0;0];
    roll    = atand(V^2/(trajSpec.R*g));
    pitch   = 0;
    heading = 0;
    accelreq = sqrt((V^2/trajSpec.R)^2 + 9.81^2);
    alpha_trim   = accelreq/dGdAlpha;
    
    % Component about y axis.
    expected_pitch_rate = sind(roll)*V/trajSpec.R;
end

DCM = EulerToDCM_ENU(deg2rad(roll),deg2rad(pitch),deg2rad(heading));

if (0)
     % Rotate about y by initial angle of attack.
     T2 = [cos(alpha), 0, -sin(alpha);
                    0, 1,           0;
           sin(alpha), 0,  cos(alpha)];

     DCM = DCM*T2;
end

[roll,pitch,heading] = DCMToEuler_ENU(DCM);

q   = DCMToQuaternion(DCM');

dt = 0.01;

plotFlag = true;

N = 5000;
Pos       = zeros(N,3);
Vel       = zeros(N,3);
V_rel     = zeros(N,3);
Target    = zeros(N,3);
Accel     = zeros(N,3);
ZDir      = zeros(N,3);
Rates     = zeros(N,3);
AlphaBeta = zeros(N,2);
TargetAccel =  zeros(N,3);

iBeta = 0.8;

tShow = 14.11;

for iT=1:N
    
    DCM = QuaternionToDCM(q)';
    
    plotFlag = tShow == iT*dt;
    [pitch_rate, roll_rate, target_pos, target_accel] = calculate_guidance(DCM, pos, vel, g, plotFlag, trajSpec);

    body_rates = [roll_rate; -pitch_rate; 0];

    % Apparent velocity.
    wind = [0;-Vw*pos(3)/10;0];
    vel_air = wind - vel;
    v_rel = DCM'*vel_air;
    
    % Calculate angle of attack.
    
    alpha = atan2(v_rel(3),-v_rel(1));
    
    % Calculate sideslip angle and apply yaw rate.
    beta = atan2(v_rel(2), sqrt(v_rel(1)^2 + v_rel(3)^2));
    
    iBeta = iBeta + beta;
    body_rates(3) = -3*beta - 0.5*iBeta;

    q_rates = BodyRatesToQuaternionRates(body_rates, q);
   
   
    % Determine lift force direction.
    % Perpendicular to velocity and to aircraft y axis.
    lift_vec = cross(-vel_air, DCM(:,2));
    lift_vec = lift_vec/norm(lift_vec);
    
    qdyn = 0.5*1.225*norm(vel_air)^2;
    lift = lift_vec * qdyn*CLK*alpha;
    
    drag = vel_air/norm(vel_air) * (norm(vel_air) - V);

    accel = lift + [0;0;-g] + drag;

    q = q + q_rates*dt;
    
    pos = pos+vel*dt;
    vel = vel+accel*dt;
    
    Vel(iT,:) = vel;
    V_rel(iT,:) = v_rel;
    Pos(iT,:) = pos';
    Target(iT,:) = target_pos';
    Accel(iT,:) = accel';
    ZDir(iT,:) = DCM(:,3)';
    Rates(iT,:) = body_rates';
    AlphaBeta(iT,:) = [alpha, beta];
    TargetAccel(iT,:) = target_accel;

end

for iT=1:length(Pos)
    Error(iT,1) = sqrt(min(sum((Target - Pos(iT,:)).^2,2)));
end

Time = (1:N)*dt;

figure,plot3(Pos(:,1),Pos(:,2),Pos(:,3));
hold on;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
plot3(Target(:,1), Target(:,2), Target(:,3),'r');
axis equal;

idx = find(Time==tShow,1,'first');
plot3([Pos(idx,1),Target(idx,1)],[Pos(idx,2),Target(idx,2)],[Pos(idx,3),Target(idx,3)],'r--');

figure;

subplot(3,1,1); plot(Time, sqrt(sum(Vel.^2,2)));
xlabel('Time [s]'); ylabel('Velocity [m/s]');
grid on; grid minor;

subplot(3,1,2); plot(Time, Pos(:,3));
xlabel('Time [s]'); ylabel('Height [m]');
grid on; grid minor;

subplot(3,1,3); plot(Time, rad2deg(AlphaBeta));
xlabel('Time [s]'); ylabel('Angle [deg]');
grid on; grid minor;
legend('Alpha','Beta');

figure;
plot(Time, Target(:,1), Time, Target(:,2), Time, Target(:,3));
title('Target coordinates');
xlabel('Time [s]'); ylabel('Position [m]');
grid on; grid minor;

figure;
plot(Time, Rates(:,1), Time, -Rates(:,2), Time, Rates(:,3));
title('Target body rates');
legend('Roll','Pitch','Yaw');
xlabel('Time [s]'); ylabel('Rates [rad/s]');
grid on; grid minor;

figure;
plot(Time, Accel(:,1), Time, Accel(:,2), Time, Accel(:,3));
hold on;
set(gca,'ColorOrderIndex',1);
plot(Time, TargetAccel(:,1),'--', Time, TargetAccel(:,2),'--', Time, TargetAccel(:,3),'--');
title('Accelerations');
legend('X','Y','Z');
xlabel('Time [s]'); ylabel('Accel [m/s/s]');
grid on; grid minor;

figure,
plot(Time,Error);
xlabel('Time [s]'); ylabel('Error [m]');
grid on; grid minor;

fprintf('Mean error %3.2f max %3.2f\n', mean(Error(Time>5)),max(Error(Time>5)));

