function MakeCalTraj

%    pqr0 = [ -1.7663; -1.4002; -1.5044];
    % consider 0;0;0 as top,left,back of workspace
    SetupRosView();
    armposes = readArmPoses( [1:364], '/Users/gsibley/Data/ArmCal/apc_asus_right');        
    aero_bb = GetBoundingBox( Ros2Aero(armposes) );
    ros_bb = GetBoundingBox( armposes );

    % do all work in aero frame:
    hold on;
    bb = ScaleBoundingBox( aero_bb, 0.3 );
    WayPoints = GenPath( bb );
    d =  SE3PathIntegral( WayPoints, 0.1 );

    cs = pchip( d, WayPoints );
    xx = [d(1):0.001:d(end)];
    yy = ppval(cs,xx(1:10:end));
    yy(:,end) = WayPoints(:,end);
    
    yy = Aero2Ros( yy );
    for ii = 1:size(yy,2)
        T = Cart2T_ros( yy(:,ii) );
        transforms(ii,:) = [T(1,:),T(2,:),T(3,:),T(4,:)];
    end

    csvwrite('generated_ros_poses.csv', transforms );
    
    % convert back to ros, do all drawing in ros frame to verify
    SetupRosView();
    DrawBoundingBox( ros_bb );
    DrawBoundingBox( ScaleBoundingBox( ros_bb, 0.3 ) );
    plot3( yy(1,:), yy(2,:), yy(3,:), 'b-' );
    axis tight;
    set(gcf,'position',get(0,'ScreenSize'));    
    h = plot_ros_cf( yy(:,1), 0.1 );
    for ii = 1:size(yy,2)
        delete(h);
        h = plot_ros_cf( yy(:,ii), 0.1 );
        drawnow();
    end
end

function WayPoints = GenPath( bb )
    roll = pi/12;
    pitch = pi/12;
    yaw = pi/6;

    doRoll = false;
    doPitch = false;
    doYaw = true;

    % top left waive
    WayPoints(:,1) = [0;0;0; 0;0;0];
    if doRoll
        WayPoints(:,end+1) = [0;0;0; roll;0;0];
        WayPoints(:,end+1) = [0;0;0;-roll;0;0];
        WayPoints(:,end+1) = [0;0;0; 0;0;0];
    end
    if doPitch
        WayPoints(:,end+1) = [0;0;0; 0;pitch;0];
        WayPoints(:,end+1) = [0;0;0; 0;-pitch;0];
        WayPoints(:,end+1) = [0;0;0; 0;0;0];
    end
    if doYaw
        WayPoints(:,end+1) = [0;0;0; 0;0;yaw];
        WayPoints(:,end+1) = [0;0;0; 0;0;-yaw];
        WayPoints(:,end+1) = [0;0;0; 0;0;0];
    end

    % bottom left waive
    WayPoints(:,end+1) = [0;0;bb.dz; 0;0;0];
    if doRoll
        WayPoints(:,end+1) = [0;0;bb.dz; roll;0;0];
        WayPoints(:,end+1) = [0;0;bb.dz; -roll;0;0];
        WayPoints(:,end+1) = [0;0;bb.dz; 0;0;0];
    end
    if doPitch
        WayPoints(:,end+1) = [0;0;bb.dz; 0;pitch;0];
        WayPoints(:,end+1) = [0;0;bb.dz; 0;-pitch;0];
        WayPoints(:,end+1) = [0;0;bb.dz; 0;0;0];
    end
    if doYaw
        WayPoints(:,end+1) = [0;0;bb.dz; 0;0;yaw];
        WayPoints(:,end+1) = [0;0;bb.dz; 0;0;-yaw];
        WayPoints(:,end+1) = [0;0;bb.dz; 0;0;0];        
    end

    % bottom right  waive
    WayPoints(:,end+1) = [0;bb.dy;bb.dz; 0;0;0];
    if doRoll
        WayPoints(:,end+1) = [0;bb.dy;bb.dz; 0;0;roll];
        WayPoints(:,end+1) = [0;bb.dy;bb.dz; 0;0;-roll];
        WayPoints(:,end+1) = [0;bb.dy;bb.dz; 0;0;0];
    end
    if doPitch
        WayPoints(:,end+1) = [0;bb.dy;bb.dz; 0;pitch;0];
        WayPoints(:,end+1) = [0;bb.dy;bb.dz; 0;-pitch;0];
        WayPoints(:,end+1) = [0;bb.dy;bb.dz; 0;0;0];
    end
    if doYaw
        WayPoints(:,end+1) = [0;bb.dy;bb.dz; 0;0;yaw];
        WayPoints(:,end+1) = [0;bb.dy;bb.dz; 0;0;-yaw];
        WayPoints(:,end+1) = [0;bb.dy;bb.dz; 0;0;0];
    end
    
    % top right  waive
    WayPoints(:,end+1) = [0;bb.dy;0; 0;0;0];
    if doRoll
        WayPoints(:,end+1) = [0;bb.dy;0; roll;0;0];
        WayPoints(:,end+1) = [0;bb.dy;0;-roll;0;0];
        WayPoints(:,end+1) = [0;bb.dy;0; 0;0;0];
    end
    if doPitch
        WayPoints(:,end+1) = [0;bb.dy;0; 0;pitch;0];
        WayPoints(:,end+1) = [0;bb.dy;0; 0;-pitch;0];
        WayPoints(:,end+1) = [0;bb.dy;0; 0;0;0];
    end
    if doYaw
        WayPoints(:,end+1) = [0;bb.dy;0; 0;0;yaw];
        WayPoints(:,end+1) = [0;bb.dy;0; 0;0;-yaw];
        WayPoints(:,end+1) = [0;bb.dy;0; 0;0;0];
    end
        
    
    % back to top left
    WayPoints(:,end+1) = [0;0;0; 0;0;0];

    WayPoints(1,:) = WayPoints(1,:) + bb.min_x;
    WayPoints(2,:) = WayPoints(2,:) + bb.min_y;
    WayPoints(3,:) = WayPoints(3,:) + bb.min_z;
end
