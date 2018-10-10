function trajectoryPlot(x,y,z,vx,vy,vz,heading, roll, windFunc, vd)
% Plotting

    if nargin<10 || isempty(vd)
        vd = struct();
    end
    
    figure,hold on;

    plot3(x,y,z,'k:');
    plot3(0*x+max(x),y,z,'g');
    plot3(x,0*y+max(y),z,'b');
    plot3(x,y,0*z+min(z),'r');
    view(3);
    grid on; grid minor;

    % Velocities
    idx = round(linspace(1,length(x),10));
    quiver3(x(idx),y(idx),z(idx),vx(idx),vy(idx),vz(idx),0.5);

    if isfield(vd,'acx')
        % quiver3(x(idx),y(idx),z(idx),vd.acx(idx),vd.acy(idx),vd.acz(idx),0.2,'r');
    end
    if isfield(vd,'Fx')
        quiver3(x(idx),y(idx),z(idx),vd.Fx(idx),vd.Fy(idx),vd.Fz(idx),0.2,'g');

        quiver3(x(idx),y(idx),z(idx),vd.V_rel(1,idx),     vd.V_rel(2,idx),     vd.V_rel(3,idx),0.5,'k');
        quiver3(x(idx),y(idx),z(idx),vd.liftFvec(1,idx),  vd.liftFvec(2,idx),  vd.liftFvec(3,idx),0.2,'m');
        quiver3(x(idx),y(idx),z(idx),vd.dragFvec(1,idx),  vd.dragFvec(2,idx),  vd.dragFvec(3,idx),0.2,'c');
        quiver3(x(idx),y(idx),z(idx),vd.thrustFvec(1,idx),vd.thrustFvec(2,idx),vd.thrustFvec(3,idx),0.2,'r');
    end
    
    % quiver3(0*x+min(x),y,z,0*vx,vy,vz);
    % quiver3(x,0*y+min(y),z,vx,0*vy,0*vz);
    % quiver3(x,y,0*z+min(z),vx,vy,0*vz);

    plotSmallAeroplanes(gca,x,y,z,heading,roll,0*roll,idx);

    axis equal;

    % legend('','','','','Vel','Acc','F','Vrel','L','D');
    legend('','','','','Vel','F','Vrel','L','D','T');

    zw = linspace(0.1,max(z),20);
    wnd = windFunc(0*zw,0*zw,zw);
    quiver3(0*zw+max(x),0*zw+max(y),zw,wnd(1,:),wnd(2,:),wnd(3,:));

    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
end
