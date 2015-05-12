function TestROSPoses

    % test inverse
    x = randn(6,1)*0.5;
%    x = [0;0;0; 0.1;0.1;0.1];
    T = Cart2T_ros( x );
    err = T2Cart_ros(T) - x;
    if norm(err) > 1e-6
        error('Error in T2Cart_ros or Cart2T_ros');
    end

    SetupRosView();

    xlim([-2,2]);
    ylim([-2,2]);
    zlim([-2,2]);

    dth = 0.1;

    % make sure roll looks right:
    t = text(0,0,-0.1,'Testing Roll');
    h = plot_ros_cf( [0;0;0;0;0;0] );
    for th = 0:dth:2*pi+dth
        delete(h);
        h = plot_ros_cf( [0;0;0;th;0;0] );
        drawnow;
    end
    delete(t);
    
    % make sure pitch looks right
    t = text(0,0,-0.1,'Testing Pitch');
    for th = 0:dth:2*pi+dth
        delete(h);
        h = plot_ros_cf( [0;0;0;0;th;0] );
        drawnow;
    end
    delete(t);

    % make sure yaw looks right
    t = text(0,0,-0.1,'Testing Yaw');
    for th = 0:dth:2*pi+dth
        delete(h);
        h = plot_ros_cf( [0;0;0;0;0;th] );
        drawnow;
    end
    delete(t);
end


