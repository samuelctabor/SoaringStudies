function h = plotSmallAeroplanes(ax, x, y, z, heading, roll, pitch, idx)
    % For it=1:length(x)
    if nargin>7 && ~isempty(idx)
        x = x(idx);
        y = y(idx);
        z = z(idx);
        heading = heading(idx);
        roll    = roll(idx);
        pitch   = pitch(idx);
    end
    
    
    % Plot a series of patches.

    wing = [[0.5, 0.5, 0.35, -0.35, -0.5, -0.5]*0.2;
            [0.0, 1.0, 2.0, 2.0,  1.0,  0.0]/3;
            [0.0, 0.0, 0.05, 0.05, 0.0,  0.0]];
        
        
    wing_tip = [[0.35, 0.2, -0.2, -0.35]*0.2;
                [2.0, 3.0,  3.0,  2.0]/3;
                [0.05, 0.1,  0.1,  0.05]];
            
    tail = [[0.5, 0.5, 0.2, -0.2, -0.5, -0.5]*0.3;
            [0.0, 1.0, 3.0,  3.0,  1.0,  0.0]/3;
            [0.0, 0.0, 0.0,  0.0,  0.0,  0.0]];
    
    fuse = [[-0.80, 0.0, 0.40, 0.40, 0.0,-0.80];
            [ 0.05, 0.1, 0.08,-0.08,-0.1,-0.05]*0.3;
            [ 0.00, 0.0, 0.00, 0.00, 0.0, 0.00]];
    
    nv = length(wing);
    
    
    for i=1:length(x)
        P = [x(i); y(i); z(i)];
        
        % Rotate about x by roll.
        T1 = [1,         0 ,         0;
              0,  cos(roll(i)),-sin(roll(i));
              0,  sin(roll(i)), cos(roll(i))];

        % Rotate about y by pitch.
        T2 = [cos(pitch(i)), 0, -sin(pitch(i));
                          0, 1,              0;
              sin(pitch(i)), 0,  cos(pitch(i))];

        % Rotate about z by heading.
        T3 = [cos(heading(i)), sin(heading(i)), 0; 
             -sin(heading(i)), cos(heading(i)), 0;
                            0,               0, 1];

        T = T3*T2*T1;
        
        RY = [1, 0, 0;
              0,-1, 0;
              0, 0, 1];

        sc= 3;
        
        wing1_t = P + sc*T*wing;
        wing2_t = P + sc*T*RY*wing;

        wing1tip_t = P + sc*T*wing_tip;
        wing2tip_t = P + sc*T*RY*wing_tip;
        
        tail1_t = P + sc*T*   (tail*0.3 + [-0.8;0;0]);
        tail2_t = P + sc*T*RY*(tail*0.3 + [-0.8;0;0]);

        vtail_t = P + sc*T*   ([1,0,0;0,0,-1;0,1,0]*tail*0.3 + [-0.8;0;0]);
        
        fuse_t = P + sc*T*fuse;

        patch(ax, wing1_t(1,:), wing1_t(2,:), wing1_t(3,:), zeros(1,nv),'FaceColor','w');
        patch(ax, wing2_t(1,:), wing2_t(2,:), wing2_t(3,:), zeros(1,nv),'FaceColor','w');
        patch(ax, wing1tip_t(1,:), wing1tip_t(2,:), wing1tip_t(3,:), zeros(1,length(wing_tip)),'FaceColor','r');
        patch(ax, wing2tip_t(1,:), wing2tip_t(2,:), wing2tip_t(3,:), zeros(1,length(wing_tip)),'FaceColor','g');
        
        patch(ax, tail1_t(1,:), tail1_t(2,:), tail1_t(3,:), zeros(1,length(tail)),'FaceColor','w');
        patch(ax, tail2_t(1,:), tail2_t(2,:), tail2_t(3,:), zeros(1,length(tail)),'FaceColor','w');
        patch(ax, vtail_t(1,:), vtail_t(2,:), vtail_t(3,:), zeros(1,length(tail)),'FaceColor','w');
        patch(ax,  fuse_t(1,:),  fuse_t(2,:),  fuse_t(3,:), zeros(1,length(fuse)),'FaceColor','w');
    end
end