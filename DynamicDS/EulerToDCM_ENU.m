function T=EulerToDCM_ENU(roll, pitch, heading)
    % Rotate about x by roll.
    T1 = [1,         0 ,         0;
          0,  cos(roll), -sin(roll);
          0,  sin(roll),  cos(roll)];

    % Rotate about y by pitch.
    T2 = [cos(pitch), 0, -sin(pitch);
        0, 1,              0;
        sin(pitch), 0,  cos(pitch)];

    % Rotate about z by heading.
    T3 = [cos(heading),-sin(heading), 0;
        sin(heading), cos(heading), 0;
        0,               0, 1];

    T = T3'*T2*T1;
end