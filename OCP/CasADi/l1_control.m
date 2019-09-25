function [ roll_target ] = l1_control( current_loc, centre_wp, heading, velocity, L1, radius)
% L1 control
%   % Lateral control based on L1 distance.
    % Based on AP_L1_Control.cpp and
    % S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
    % Proceedings of the AIAA Guidance, Navigation and Control
    % Conference, Aug 2004. AIAA-2004-4900.
    loiter_dir = 1;
    
    K_L1 = 4*0.75*0.75;
    
    % Groundspeed
    groundSpeed_vector = [cos(heading); sin(heading)]*velocity;
    
    % Vector from WP to aircraft
    A_air = current_loc - centre_wp;
    
    A_air_unit = A_air / norm(A_air);
    
    xTrackVelCap = cross([A_air_unit;0], [groundSpeed_vector;0]);
    lTrackVelCap = - dot( groundSpeed_vector, A_air_unit);
    Nu = atan2(xTrackVelCap(3), lTrackVelCap);
    
    % Rather than use the PD control law, calculate the angle offset due to
    % the target point moving round the circle. In steady-state circling
    % this term reduces to L1/(2*r), so if L1 is more that 2*r it becomes
    % capped at 1, and Nu1 is 
    cosTerm = (L1^2 + norm(A_air)^2 - radius*radius)/(2*L1*norm(A_air));
    cosTerm = max(min(cosTerm, 1.0), -1.0);
    Nu1 = acos(cosTerm);
    
    Nu = Nu + loiter_dir*Nu1;
    
    latAccDemCap = K_L1 * velocity^2 / L1 * sin(Nu);
    
    roll_target = atan(latAccDemCap/9.81);
end

