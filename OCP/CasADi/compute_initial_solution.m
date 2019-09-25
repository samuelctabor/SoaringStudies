function [ Sol ] = compute_initial_solution( start_pos, start_heading, target, V, r, N, T )
%compute_initial_solutions Generate a feasible initial trajectory using L1
%control method.

    pos = start_pos;
    heading = start_heading;
    
    dt = T/N;
    
    Sol = zeros(N,3);
    Sol(1,:) = [pos(1), pos(2), heading];
    
    for i=2:N        
        roll = l1_control(pos, target, heading, V, 1.5*V, r);
        
        pos(1)  = pos(1)  + cos(heading)*V  * dt;
        pos(2)  = pos(2)  + sin(heading)*V  * dt;
        heading = heading + 9.8*sin(roll)/V * dt;
        
        Sol(i, :) = [pos(1), pos(2), heading];
    end
end