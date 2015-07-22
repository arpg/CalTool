function RegisterTrajectories( pg1, pg2 )
    global ctx;
    ctx = [];
    ctx.traj_c1 = pg1;
    ctx.traj_c2 = pg2;
    ctx.x = [ 0.1511; -0.1032; 0.0291; 0.0017; 0.0007; 0.0247; 1 ];
    ctx.x = [0.1, 0.2, 0.3, 0.1, 0.2, 0.3, 5]';    

    %ctx.x = [4.93128, 0.473484, -0.369487, ...
    %        -0.0357445, -0.00755197, 1.22677, 5.36593]';
    
    %'Correct' answer
    %ctx.x = [-0.0501    0.4766   -0.1261    1.2272   -0.0233   -0.0251    5.2743]';
    plot_system( ctx );
    opts = optimset('Display', 'Iter');
    %, ,,'Algorithm', 'levenberg-marquardt'
    [ctx.x, ctx.resnorm, ctx.resval,~,~,~,ctx.J]  = lsqnonlin( @residual_noJ, ctx.x, [], [], opts );
%    ctx.x = fminsearch( @rtr, ctx.x, opts );
    ctx.x'
    
    %Estimate covariance as J'*J
    
    disp('Variances:');
    diag(inv(full(ctx.J'*ctx.J)))
end

function e = rtr( x )
    e = residual(x).'*residual(x);
end

function [r, J] = residual(x)

    [r, J ] = FiniteDiff(@residual_noJ, x);
    
end

function r = residual_noJ( x )
    global ctx;
    ctx.x = x;
    n = size(ctx.traj_c1,2);
    Tab = Cart2T_vis(x(1:6)); 
%     Tab(1:3,1:3) = Tab(1:3,1:3)*x(7);
%     r = [];
%     for ii = 1:n
%         r = [r; ctx.traj_c1(1:3,ii) - Tab(1:3,:)*[ctx.traj_c2(1:3,ii);1]];
%     end

    %Tab(1:3,4) = Tab(1:3,4);
    r = [];
    for ii = 1:n
        r = [r; ctx.traj_c1(1:3,ii) - Tab(1:3,:)*[ctx.traj_c2(1:3,ii).*x(7);1]];
    end
    plot_system( ctx );
    %waitforbuttonpress;
end

function plot_system( ctx )
    clf;
    hold on; axis vis3d; grid on; view(3);
    n = size(ctx.traj_c1,2);
    T = Cart2T_vis(ctx.x);
%     T(1:3,1:3) = T(1:3,1:3)*ctx.x(7); %should the scale factor be applied
%     %to the rotation matrix?
    
    xyz = ctx.traj_c1(1:3,:);
    plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'b-', 'LineWidth', 2 );

    xyz = T(1:3,:)*[ctx.traj_c2(1:3,:)*ctx.x(7); ones(1,n)]; %scale applied to xyz
    %xyz = T(1:3,:)*[ctx.traj_c2(1:3,:); ones(1,n)]; %scale applied to
    %rotation matrix
    h = plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'r-', 'LineWidth', 2 );

    legend('Camera Poses', 'Arm Poses');

    for ii = 1:n
       xyz = [ctx.traj_c1(1:3,ii)'; [T(1:3,:)*[ctx.traj_c2(1:3,ii);1]]'];
       line( xyz(:,1),xyz(:,2),xyz(:,3), 'LineStyle',':', 'Color', 'c' );
       if (mod(ii,10) == 0 && (ii < 50))
        plot_vis_cf(ctx.traj_c1(1:6,ii),1);
        plot_vis_cf(T2Cart_vis(T*Cart2T_vis(ctx.traj_c2(1:6,ii))),1);
       end
    end
    axis tight;
    drawnow();
end
