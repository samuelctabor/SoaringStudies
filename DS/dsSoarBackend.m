function [thrustPower, CL, CD, CT, x, y, z, vx, vy, vz, psi, roll, vectorData] = ...
    dsSoarBackend(t,windFunc,traj,params,conditions,f)

    norm2 = @(v) sqrt(sum(v.^2,1));
    
    N = length(traj.ax);

    if nargin>5 && ~isempty(f)
        w = 2*pi*f;
    else
        w = 2*pi*(1:N)'/max(t);
    end
    
    x = traj.Vnet*cos(traj.phi)*cos(traj.th)*t + sum(traj.ax.*sin(w*t + traj.bx),1);
    y = traj.Vnet*sin(traj.phi)*cos(traj.th)*t + sum(traj.ay.*sin(w*t + traj.by),1);
    z = traj.Vnet*              sin(traj.th)*t + sum(traj.az.*sin(w*t + traj.bz),1);
    
    z = z - min(z) + traj.clz0;

    % Derive other quantities from these differentially flat outputs and the
    % system dynamics.
    vx = traj.Vnet*cos(traj.phi)*cos(traj.th) + sum(traj.ax.*cos(w*t + traj.bx).*w,1);
    vy = traj.Vnet*sin(traj.phi)*cos(traj.th) + sum(traj.ay.*cos(w*t + traj.by).*w,1);
    vz = traj.Vnet              *sin(traj.th) + sum(traj.az.*cos(w*t + traj.bz).*w,1);

    acx =                                     - sum(traj.ax.*sin(w*t + traj.bx).*w.^2,1);
    acy =                                     - sum(traj.ay.*sin(w*t + traj.by).*w.^2,1);
    acz =                                     - sum(traj.az.*sin(w*t + traj.bz).*w.^2,1);

    Fx = acx*params.m;
    Fy = acy*params.m;
    Fz = acz*params.m + params.m*conditions.g;

    %
    % Now solve for angle of attack and roll angle. 
    %
    % Find apparent wind.
    V_rel = windFunc(x,y,z) - [vx;vy;vz];

    % Assume zero sideslip.
    psi = atan2(V_rel(2,:), -V_rel(1,:));
    
    qS = 0.5*conditions.rho*norm2(V_rel).^2*params.S;

    % Find the total aerodynamic force coefficient.
    Fvec = [Fx;Fy;Fz];
    Ctot = norm2(Fvec)./qS;

    % % Find angle of attack, CL and CD.
    % Ctot_in = sqrt(sum(params.CL.^2+params.CD.^2,1));

    % Find stall angle index.
    [~,index] = max(params.CL);
    idx = 1:index;

    % alpha = interp1(Ctot_in(idx), params.alpha(idx), Ctot);
    % CL = interp1(params.alpha, params.CL, alpha);
    % CD = interp1(params.alpha, params.CD, alpha);

    % Find the aerodynamic force component aligned with V_rel. This is the
    % drag.
    V_rel_norm = V_rel./norm2(V_rel) ;
    fwdFmag = dot(V_rel_norm, Fvec);
    dragFvec = fwdFmag.*V_rel_norm;
    CDfwd = fwdFmag./qS;

    % Everything else is lift.
    liftFvec = Fvec - dragFvec; 
    liftFmag = norm2(liftFvec);
    CL = liftFmag./qS;

    % Now we can determine roll angle.
    local_horiz=cross(repmat([0;0;1],1,length(V_rel_norm)),V_rel_norm,1);
    lat_lift = dot(liftFvec, local_horiz./norm2(local_horiz));
    roll = asin(lat_lift./liftFmag);
%     roll = atan2(norm2(liftFvec(1:2,:)),liftFvec(3,:));
   
    % Find alpha for this lift.
    alpha = interp1(params.CL(idx),params.alpha(idx),CL);

    % Drag coefficient.
    CD = interp1(params.alpha,params.CD,alpha);

    % Thrust required.
    CT = CD - CDfwd;
    dragFvec   =  qS.*CD.*V_rel;
    thrustFvec = -qS.*CT.*V_rel;
    thrustFvecmag = norm2(thrustFvec);
    
%     thrustPower = trapz(t,qS.*abs(CT).*norm2(V_rel));
%     thrustPower = trapz(t,(qS.*CT.^2.*norm2(V_rel)));
    CT_int = CT;
    
    % CT is used as cost function. Add penalty for violating CLmax and
    % CLmin constraints.
    CLmax = 1.52;
    CLmin = min(params.CL);
    idx = CL>CLmax | CL<CLmin;
    CT_int(idx) = 10*(max(CL(idx)-CLmax,0)).^2 + 10*(max(CLmin-CL(idx),0)).^2;
    
    thrustPower = trapz(t,(qS.*CT_int.*norm2(V_rel)).^2);
    
    % Vector data.
    vectorData.Fx = Fx;
    vectorData.Fy = Fy;
    vectorData.Fz = Fz;
    vectorData.acx = acx;
    vectorData.acy = acy;
    vectorData.acz = acz;
    vectorData.liftFvec = liftFvec;
    vectorData.dragFvec = dragFvec;
    vectorData.thrustFvec = thrustFvec;
    vectorData.V_rel     = V_rel;
end