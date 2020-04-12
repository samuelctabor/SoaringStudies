function T = QuaternionToDCM(q)
%QuaternionToDCM Convert a quaternion into a DCM transformation matrix.
%    See http://renaissance.ucsd.edu/courses/mae207/wie_chap5.pdf eq. 5.36.

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    
    T = zeros(3,3);

    T = [1-2*(q2^2  + q3^2),    2*(q1*q2 + q3*q4),   2*(q1*q3 - q2*q4);
           2*(q2*q1 - q3*q4), 1-2*(q1^2  + q3^2),    2*(q2*q3 + q1*q4);
           2*(q3*q1 + q2*q4),   2*(q3*q2 - q1*q4), 1-2*(q1^2  + q2^2)];

end

