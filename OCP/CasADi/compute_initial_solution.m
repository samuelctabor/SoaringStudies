function [ Sol ] = compute_initial_solution( start_pos, start_heading, target, V, r, N, T )
%compute_initial_solutions Generate a feasible initial trajectory using L1
%control method.
%   This uses roll as the control variable, whereas more complex methods
%   will use the roll rate. Therefore roll is returned as part of the state
%   vector and roll rate is calculated and returned as the control.

    pos = start_pos;
    heading = start_heading;
    
    dt = T/N;
    
    Sol.X = zeros(N,4);
    Sol.U = zeros(N,1);
    
    Sol.X(1,:) = [pos(1), pos(2), heading, 0];
    Sol.U(1)   = 0;
    
    for i=2:N        
        roll = l1_control(pos, target, heading, V, 1.5*V, r);
        
        pos(1)  = pos(1)  + cos(heading)*V  * dt;
        pos(2)  = pos(2)  + sin(heading)*V  * dt;
        heading = heading + 9.8*sin(roll)/V * dt;
        
        Sol.X(i, :) = [pos(1), pos(2), heading, roll];
    end
    
    Sol.U(2:N) = diff(Sol.X(:,4))/dt;
end