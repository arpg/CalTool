function TestVisionPoses

    % test inverse
    x = 0.1*randn(6,1);
    x = [0;0;0;0;0.1;0];
    T = Cart2T_vis( x );

    err = T2Cart_vis(T) - x;
    if norm(err) > 1e-6
        error('Error in T2Cart or Cart2T');
    end

    SetupVisionView();

    xlim([-2,2]);
    ylim([-2,2]);
    zlim([-2,2]);
    


    dth = 0.1;
    h = plot_vis_cf( [0;0;0;0;0;0] );
    
    % make sure roll looks right:
    t = text(0,0,-0.1,'Testing Roll');
    for th = 0:dth:2*pi+dth
        delete(h);
        h = plot_vis_cf( [0;0;0;th;0;0] );
        drawnow;
    end
    delete(t);
    
    % make sure pitch looks right
    t = text(0,0,-0.1,'Testing Pitch');
    for th = 0:dth:2*pi+dth
        delete(h);
        h = plot_vis_cf( [0;0;0;0;th;0] );
        drawnow;
    end
    delete(t);

    % make sure yaw looks right
    t = text(0,0,-0.1,'Testing Yaw');
    for th = 0:dth:2*pi+dth
        delete(h);
        h = plot_vis_cf( [0;0;0;0;0;th] );
        drawnow;
    end
    delete(t);
end
