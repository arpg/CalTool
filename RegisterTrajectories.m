function RegisterTrajectories( pg1, pg2 )
    global ctx;
    ctx = [];
    ctx.traj_c1 = pg1;
    ctx.traj_c2 = pg2;
    ctx.x = [ 0.1511; -0.1032; 0.0291; 0.0017; 0.0007; 0.0247; 1 ];
    
    
    
    plot_system( ctx );
    opts = optimset('Display', 'Iter');
    ctx.x = lsqnonlin( @residual, ctx.x, [], [], opts );
%    ctx.x = fminsearch( @rtr, ctx.x, opts );
    ctx.x'
end

function e = rtr( x )
    e = residual(x).'*residual(x);
end

function r = residual( x )
    global ctx;
    ctx.x = x;
    n = size(ctx.traj_c1,2);
    Tab = Cart2T_vis(x(1:6));
    Tab(1:3,1:3) = Tab(1:3,1:3)*x(7);
    r = [];
    for ii = 1:n
        r = [r; ctx.traj_c1(1:3,ii) - Tab(1:3,:)*[ctx.traj_c2(1:3,ii);1]];
    end
    plot_system( ctx );
%    waitforbuttonpress;
end

function plot_system( ctx )
    clf;
    hold on; axis vis3d; grid on; view(3);
    n = size(ctx.traj_c1,2);
    T = Cart2T_vis(ctx.x);
    T(1:3,1:3) = T(1:3,1:3)*ctx.x(7);
    xyz = ctx.traj_c1(1:3,:);
    plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'b-', 'LineWidth', 2 );

    xyz = T(1:3,:)*[ctx.traj_c2(1:3,:); ones(1,n)];
    h = plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'r-', 'LineWidth', 2 );

    legend('Camera Poses', 'Arm Poses');

    for ii = 1:n
       xyz = [ctx.traj_c1(1:3,ii)'; [T(1:3,:)*[ctx.traj_c2(1:3,ii);1]]'];
       line( xyz(:,1),xyz(:,2),xyz(:,3), 'LineStyle',':', 'Color', 'c' );
    end
    axis tight;
    drawnow();
end
