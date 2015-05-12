function TestConvertPathAeroToRos()
   
    bb = GetBoundingBox( [[-2:2];[-2:2];[-2:2]] );
    p = GenPath( bb );
    d =  SE3PathIntegral( p, 1 );

    cs = pchip( d, p );
    xx = [d(1):0.001:d(end)];
    yy = ppval(cs,xx(1:100:end));

    SetupRosView();
    yy = ConvertPathAeroToRos( yy );
    plot3( yy(1,:), yy(2,:), yy(3,:), 'b-' );
    axis tight;
    set(gcf,'position',get(0,'ScreenSize'));    
    h = plot_ros_cf( yy(:,1) );
    for ii = 1:size(yy,2)
        delete(h);
        h = plot_ros_cf( yy(:,ii) );
        drawnow();
    end       
end

function WayPoints = GenPath( bb )
    
    roll = pi/12;
    pitch = pi/12;
    yaw = pi/6;

    doRoll = true;
    doPitch = true;
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
end