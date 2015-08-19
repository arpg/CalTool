function RegisterTrajectories( pg1, pg2 )
    global ctx;
    ctx = [];
    ctx.traj_c1 = pg1;
    ctx.traj_c2 = pg2;
    ctx.x = [0.1, 0.2, 0.3, 0.1, 0.2, 0.3, 0.5]';    
    
    %'Correct' answer
    %ctx.x = [-0.0501    0.4766   -0.1261    1.2272   -0.0233   -0.0251    5.2743]';
    %ctx.x = [0.0286   -0.0213    0.0291    1.2615   -0.0468   -0.0356    0.4985]';
    
    plot_system( ctx );
    opts = optimset('Display', 'Iter');
    %, ,,'Algorithm', 'levenberg-marquardt'
    [ctx.x, ctx.resnorm, ctx.resval,~,~,~,ctx.J]  = lsqnonlin( @residual_noJ, ctx.x, [], [], opts );
%    ctx.x = fminsearch( @rtr, ctx.x, opts );
    ctx.x'
    
    %Estimate covariance as J'*J
    
    disp('Variances:');
    diag(inv(full(ctx.J'*ctx.J)))
    
    %disp('Partial residuals');
    %ctx.resval(1:20)
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
        %Compose the residual using simple subtraction on (x,y,z)
        %total_res = abs(ctx.traj_c1(1:3,ii) - Tab(1:3,:)*[ctx.traj_c2(1:3,ii).*x(7);1]);
        %r = [r; total_res];
    
        %Compose using Tc1 * inv(pred_c1), where pred_c1 is Tab scaled
        %W/o angular info
        
        %master_traj = Cart2T_vis([ctx.traj_c1(1:3,ii); 0;0;0]);
        %slave_traj = Cart2T_vis([ctx.traj_c2(1:3,ii).*x(7); 0;0;0]) * inv(Tab);
        
        %With angles using inv()
        %master_traj = Cart2T_vis(ctx.traj_c1(:,ii));
        %slave_traj = Cart2T_vis([ctx.traj_c2(1:3,ii).*x(7); ctx.traj_c2(4:6,ii)]) * inv(Tab);
        %total_res = abs(T2Cart_vis(master_traj*inv(slave_traj)));
        %r = [r; total_res];
        

        %For some reason, the <roll> component of the parameter vector
        %(x[4]) has a large residual that strikingly corresponds to the
        %correct rotation parameter. 
        
        %Using just subtraction, not running the angles through the
        %transform:
        %This works, but still doesn't yield the correct angles at the end
        %of the optimization
        %projC2_xyz = Tab*[ctx.traj_c2(1:3,ii).*x(7); 1];
        
        %then, rotate the angles through the transform separately
        %projC2_rpy = T2Cart_vis(Tab*Cart2T_vis([0 0 0 ctx.traj_c2(1:3,ii)']'));
        
        %total_res = abs(ctx.traj_c1(1:6,ii) - [projC2_xyz(1:3,:); ctx.traj_c2(4:6,ii)]);
        %total_res = abs(ctx.traj_c1(1:6,ii) - [projC2_xyz(1:3,:); projC2_rpy(4:6,:)]);
        
        %Using just subtraction
        %Compute the initial offset that has to be taken out at the end
        %Without this, the transform Tab double-counts the rotations
        T_rpy = Cart2T_vis([0; 0; 0; x(4:6)]);
        
        total_res = abs(ctx.traj_c1(1:6,ii) - T2Cart_vis(Tab*Cart2T_vis([ctx.traj_c2(1:3,ii).*x(7);ctx.traj_c2(4:6,ii)])*inv(T_rpy)));
  
        r = [r; total_res];
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
%     %to the rotation matrix? - NO!
    
    xyz = ctx.traj_c1(1:3,:);
    plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'b-', 'LineWidth', 2 );

    xyz = T(1:3,:)*[ctx.traj_c2(1:3,:)*ctx.x(7); ones(1,n)]; %scale applied to xyz
    %xyz = T(1:3,:)*[ctx.traj_c2(1:3,:); ones(1,n)]; %scale applied to
    %rotation matrix
    h = plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'r-', 'LineWidth', 2 );

    legend('Camera Poses', 'Arm Poses');

    for ii = 1:n
       xyz = [ctx.traj_c1(1:3,ii)'; [T(1:3,:)*[ctx.traj_c2(1:3,ii).*ctx.x(7);1]]'];
       line( xyz(:,1),xyz(:,2),xyz(:,3), 'LineStyle',':', 'Color', 'c' );
       if (mod(ii,10) == 0)
           %plot the angular data as well
            plot_vis_cf(ctx.traj_c1(1:6,ii),1);
  
            %Take out the initial angular offset that represents the
            %transform we're trying to find
            
            T_rpy = Cart2T_vis([0; 0; 0; ctx.x(4:6)]);
            plot_vis_cf(T2Cart_vis(T*Cart2T_vis([ctx.traj_c2(1:3,ii).*ctx.x(7); ctx.traj_c2(4:6,ii)])*inv(T_rpy)),1);
       end
    end
    axis tight;
    drawnow();
end
