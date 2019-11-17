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

% Assume 3 degrees alpha for level 1G flight at V.
dGdAlpha = g/deg2rad(3);

% Initial conditions
% 20 deg inclination, small error
if (0)
    trajSpec.inclination = 20;
    trajSpec.R = 30;
    pos = [0;30;30];
    vel = [V;0;0];
    roll    = 90;
    pitch   = 0;
    heading = 0;
    alpha   = 3;
else
    trajSpec.inclination = 0;
    trajSpec.R = 30;
    pos = [0; trajSpec.R; 1.5];
    vel = [V;0;0];
    roll    = atand(V^2/(trajSpec.R*g));
    pitch   = 0;
    heading = 0;
    accelreq = sqrt((V^2/trajSpec.R)^2 + 9.81^2);
    alpha   = accelreq/dGdAlpha;
end

DCM = EulerToDCM_ENU(deg2rad(roll),deg2rad(pitch),deg2rad(heading));

% Rotate about y by initial angle of attack.
 T2 = [cos(alpha), 0, -sin(alpha);
                0, 1,           0;
       sin(alpha), 0,  cos(alpha)];

 DCM = DCM*T2;

[roll,pitch,heading] = DCMToEuler_ENU(DCM);

q   = DCMToQuaternion(DCM');

dt = 0.01;

plotFlag = true;

N = 2000;
Pos       = zeros(N,3);
Target    = zeros(N,3);
Accel     = zeros(N,3);
ZDir      = zeros(N,3);
Rates     = zeros(N,3);
AlphaBeta = zeros(N,2);

for iT=1:N
    
    DCM = QuaternionToDCM(q)';
    
    plotFlag = 0;%mod(iT,100)==0;
    [pitch_rate, roll_rate, target_pos] = calculate_guidance(DCM, pos, vel, g, plotFlag, trajSpec);

    body_rates = [roll_rate; -pitch_rate; 0];

    % Apparent velocity.
    wind = [0;0;0];
    vel_air = wind - vel;
    v_rel = DCM'*vel_air;
    
    % Calculate angle of attack.
    
    alpha = atan2(v_rel(3),-v_rel(1));
    
    % Calculate sideslip angle and apply yaw rate.
    beta = atan2(v_rel(2), sqrt(v_rel(1)^2 + v_rel(3)^2));
    
    body_rates(3) = -3*beta;

    q_rates = BodyRatesToQuaternionRates(body_rates, q);
   
   
    % Determine lift force direction.
    % Perpendicular to velocity and to aircraft y axis.
    lift_vec = cross(vel, DCM(:,2));
    lift_vec = lift_vec/norm(lift_vec);
    
    accel = lift_vec*dGdAlpha*alpha + [0;0;-g];
    
    q = q + q_rates*dt;
    
    pos = pos+vel*dt;
    vel = vel+accel*dt;
    
    Pos(iT,:) = pos';
    Target(iT,:) = target_pos';
    Accel(iT,:) = accel';
    ZDir(iT,:) = DCM(:,3)';
    Rates(iT,:) = body_rates';
    AlphaBeta(iT,:) = [alpha, beta];
end

figure,plot3(Pos(:,1),Pos(:,2),Pos(:,3));
hold on;

plot3(Target(:,1), Target(:,2), Target(:,3),'r');
axis equal;