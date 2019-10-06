function q = DCMToQuaternion(T)
%Convert a DCM to a quaternion.
%   See http://renaissance.ucsd.edu/courses/mae207/wie_chap5.pdf eq. 5.39
%   and 5.40.

    q = zeros(4,1);
    
    q(4) = 0.5*sqrt(1 + T(1,1) + T(2,2) + T(3,3));
    
    q(1:3) = [T(2,3) - T(3,2);
              T(3,1) - T(1,3);
              T(1,2) - T(2,1)] / (4*q(4));

end

