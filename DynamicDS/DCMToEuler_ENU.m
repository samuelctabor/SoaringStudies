function [roll, pitch, heading] = DCMToEuler_ENU(T)

% Make sure none of the elements are out of range -1, 1.
T = max(min(T,1.0),-1.0);

% Roll 

pitch   = asin(T(3,1));
roll    = acos(max(min(T(3,3)/cos(pitch), 1.0), -1.0));
heading = acos(max(min(T(1,1)/cos(pitch), 1.0), -1.0));

end

