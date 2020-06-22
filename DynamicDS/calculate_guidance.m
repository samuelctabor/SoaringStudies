function [pitch_rate, roll_rate, target_pos, target_accel_earth] = calculate_guidance(DCM, pos, vel, g, plotFlag, trajSpec)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%     pitch_rate = deg2rad(10);
%     
%     roll_rate = deg2rad(10);
   
    traj.Vnet = 0;
    traj.phi  = 0;
    traj.clz0 = 1.5;
    traj.th = 0;

    traj.ax = trajSpec.R;
    traj.ay = cosd(trajSpec.inclination)*trajSpec.R;
    traj.az = sind(trajSpec.inclination)*trajSpec.R;
    traj.bx = 0;
    traj.by = pi/2;
    traj.bz = pi/2;
    traj.w  = 2*pi*(1/6);

    % Find the point on trajectory the correct distance ahead.
    L1 = 2*norm(vel);
    
    V = norm(vel);
    
    roll_tau = 0.1;
    
    t = linspace(0,6,1000);
    t  =t(1:end-1);
    
    p = trajectory_eval(traj, t);
    
    L = sqrt(sum((p - pos).^2,1));
    
    % Closest point on trajectory.
    [~,idx] = min(L);
    
    % Next point L1 distance away.
    id=1:length(t);
    id2 = [id(idx+1:end), id(1:idx)]; 
    L2 =  [L(idx+1:end),  L(1:idx)];
    
    gradient = [diff(L2),0];
    
    idx1 = find(gradient>=0 & abs(L1-L2)<0.1,1,'first');

    idx3 = id2(idx1);
    
    if isempty(idx3)
        % Occurs if no points are within tolerance
        % of L1 distance.
        idx3 = idx;
    end
    
    target_pos = p(:,idx3);
    
    % Rotate this into aircraft frame of reference.
    rel_target_pos = target_pos - pos;
    target_pos_2 = DCM'*rel_target_pos;
    
    % We need a target acceleration vector to pass through the target.
    % Angle between x axis and vector to target.
    dist = norm(target_pos_2);
%     nu = acos(dot([1;0;0], target_pos_2)/dist);
%     r = dist/(2*sin(nu));
%     acc = V^2/r;
%     accel_dir = [0; target_pos_2(2:3)];
%     accel_dir = accel_dir/norm(accel_dir);
%     target_accel = acc*accel_dir;
%     
    vel_body_norm = DCM'*vel/norm(vel);
    
    % Angle between velocity vector and target
    nu = acos(dot(vel_body_norm, target_pos_2)/dist);
    r = dist/(2*sin(nu));
    acc = V^2/r;
    accel_dir = cross(cross(vel_body_norm, target_pos_2, 1), vel_body_norm, 1);
    accel_dir = accel_dir/norm(accel_dir);
    target_accel = acc*accel_dir;
    
    % Now calculate roll angle
    gravity_rel = DCM'*[0;0;-g];
    req_lift = target_accel - gravity_rel;
    
    % Angle error
    roll_err = asin(-req_lift(2)/norm(req_lift));
    
    roll_rate = roll_err/roll_tau;
    
    % Now pitch rate
    relative_roll = asin(-target_accel(2)/norm(target_accel));
    pitch_rate = cos(relative_roll)*acc/V;

    [roll,pitch,heading] = DCMToEuler_ENU(DCM);

    target_accel_earth = DCM*target_accel;
            
    if plotFlag
        close all
        figure; hold on;
        % Plot target trajectory.
        plot3(p(1,:),p(2,:),p(3,:),'b-');
        
        % Plot closest point on trajectory.
        plot3(p(1,idx),p(2,idx),p(3,idx),'go');
        
        % Plot vector from current to target position.
        plot3([pos(1),target_pos(1)], [pos(2),target_pos(2)],[pos(3),target_pos(3)],'r-o')
        
        % Plot target acceleration vector.
        quiver3(pos(1),pos(2),pos(3),target_accel_earth(1),target_accel_earth(2),target_accel_earth(3));
        
        % Plot gravity vector.
        quiver3(pos(1),pos(2),pos(3),0,0,-g);
        
        % Plot target lift vector.
        target_lift_earth = DCM*req_lift;
        quiver3(pos(1),pos(2),pos(3),target_lift_earth(1),target_lift_earth(2),target_lift_earth(3));
        
        % Plot model
        plotSmallAeroplanes(gca, pos(1), pos(2), pos(3), heading, roll, pitch, 1);
        
        axis equal;
        
        
        xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
        view(3);
        axis tight;
        
        legend('Target traj','Target point','Vec to target','Target acc','Gravity','Target lift');
        
        % Plot in ac frame of reference.
        figure; hold on;
        
        % Plot target trajectory.
        rel_traj = DCM'*(p - pos);
        plot3(rel_traj(1,:),rel_traj(2,:), rel_traj(3,:),'b-');
        
        % Plot closest point on trajectory.
        rel_closest = DCM'*(p(:,idx)-pos);
        plot3(rel_closest(1), rel_closest(2), rel_closest(3),'go');
        
        % Plot vector from current to target position.
        plot3([0,target_pos_2(1)], [0,target_pos_2(2)],[0,target_pos_2(3)],'r-o')
        
        % Plot target acceleration vector.
        quiver3(0,0,0,target_accel(1),target_accel(2),target_accel(3));

        % Plot target lift vector.
        quiver3(0,0,0,gravity_rel(1),gravity_rel(2),gravity_rel(3));
        
        % Plot target lift vector.
        quiver3(0,0,0,req_lift(1),req_lift(2),req_lift(3));
        
        % Plot model
        plotSmallAeroplanes(gca, 0,0,0, 0,0,0, 1);
        
        axis equal;
        
        
        xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
        view(3);
        
        axis tight;
        
        legend('Target traj','Target point','Vec to target','Target acc','Gravity','Target lift');
         %End    
    end
    
%     fprintf('Roll %3.1f Pitch %3.1f Yaw %3.1f\n', rad2deg(roll), rad2deg(pitch), rad2deg(heading));
end

function p = trajectory_eval(traj, t)

    
    x = traj.Vnet*cos(traj.phi)*cos(traj.th)*t + sum(traj.ax.*sin(traj.w*t + traj.bx),1);
    y = traj.Vnet*sin(traj.phi)*cos(traj.th)*t + sum(traj.ay.*sin(traj.w*t + traj.by),1);
    z = traj.Vnet*              sin(traj.th)*t + sum(traj.az.*sin(traj.w*t + traj.bz),1);
    
    z = z - min(z) + traj.clz0;

    p = [x;y;z];
end

