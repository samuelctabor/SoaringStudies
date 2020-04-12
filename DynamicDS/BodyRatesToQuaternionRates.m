function qdot = BodyRatesToQuaternionRates( BodyRates, q )
%BodyRatesToQuaternionRates Convert body frame angular rates to rates of
%change of the the four quaternion parameters.
%   See http://renaissance.ucsd.edu/courses/mae207/wie_chap5.pdf, eq. 5.74.

    qdot = (1/2) * [q(4), -q(3),  q(2), q(1);
                    q(3),  q(4), -q(1), q(2);
                   -q(2),  q(1),  q(4), q(3);
                   -q(1), -q(2), -q(3), q(4)] * [BodyRates; 0];
               
    % Or from Satdyn_mb_2010f.pdf, where q4 is the scalar part.
%     w = BodyRates;
%     qdot = (1/2)*[   0,  w(3), -w(2), w(1);...
%                  -w(3),     0,  w(1), w(2);
%                   w(2), -w(1),     0, w(3);
%                  -w(1), -w(2), -w(3),   0] * q;

    % Enfore the constraint that sum(q.^2) = 1, see https://uk.mathworks.com/help/aeroblks/6dofquaternion.html
    K = 1;
    error = 1 - sum(q.^2);
    qdot = qdot + K * error * q;
end

