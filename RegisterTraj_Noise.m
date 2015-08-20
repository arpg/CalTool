%Include noise parameters on the trajectory information - allow for
%zero-information trajectory data

function RegisterTraj_Noise( pg1, pg2)
    global ctx;
    ctx = [];
    n = min(size(pg1,2), size(pg2,2));
    startN = 1;
    endN = 600;
    ctx.traj_c1 = pg1(:,startN:endN);
    ctx.traj_c2 = pg2(:,startN:endN);
        %Define some R matrices, one for each pose producer
    r1_sigma2_trans = 0.01;
    r1_sigma2_rot = 0.01;
    
    r2_sigma2_trans = 0.01;
    r2_sigma2_rot = 0.01;
    
    R1 = diag([r1_sigma2_trans, r1_sigma2_trans,r1_sigma2_trans, r1_sigma2_rot, r1_sigma2_rot, r1_sigma2_rot]);
    R2 = diag([r2_sigma2_trans, r2_sigma2_trans,r2_sigma2_trans, r2_sigma2_rot, r2_sigma2_rot, r2_sigma2_rot]);

    %Pretend that the second trajectory has bad z data and only yaw info:
    %ctx.traj_c2 = [ctx.traj_c2(1:2,:); zeros(1,n); ctx.traj_c2(4,:); zeros(2, n)];
    
    %Adjust the uncertainties to reflect:
    noData = 5;
    %R2 = diag([r2_sigma2_trans, r2_sigma2_trans, noData, noData, noData, r2_sigma2_rot ]);

    
    %Include the sensor noises in the context
    ctx.R1 = R1;
    ctx.R2 = R2;
       
    
    
    ctx.x = [0.1, 0.2, 0.3, 0.1, 0.2, 0.3, 0.5]';    
    %ctx.x = [0.100432, 0.191182, 0.311405, 0.0948558, 0.0330915, 0.0815505, -0.105464, -0.207696, -0.288161, 0.100465, -0.0491801, -0.00896904, 0.0443786]';
    plot_system( ctx );
    opts = optimset('Display', 'Iter');
    %, ,,'Algorithm', 'levenberg-marquardt'
    [ctx.x, ctx.resnorm, ctx.resval,~,~,~,ctx.J]  = lsqnonlin( @residual_noJ, ctx.x, [], [], opts );
%    ctx.x = fminsearch( @rtr, ctx.x, opts );
    ctx.x'
    
    %Measurement noise model:
    % Zw1c2 - zw1c1*scale*Tc1c2 = v, v is 6x1
    % v ~ N(0, R2+Tc1c2'*scale*R1*scale'*Tc1c2 )
    %Estimate covariance as inv(J'*S^(-1)*J)
    %The lsqnonlin J includes a column for the dependence on X(7) - 

   	%R block for each measurement:
    %Get the Jacobian at this value of X 
    %as a 6x7 to capture differences in scaling...
    
    [res, Jc1c2] = FiniteDiff(@noiseNorm, ctx.x );
    Runit = ctx.R2 + Jc1c2*blkdiag(ctx.R1,1)*Jc1c2'; 
    
    %Prepare a complete R block:
    meas_rz = makeBlkDiag(Runit, n);

    disp('Variances:');
    %diag(inv(full(ctx.J'*inv(meas_rz)*ctx.J)))
    
    %disp('Partial residuals');
    %ctx.resval(1:20)
end

function e = rtr( x )
    e = residual(x).'*residual(x);
end

function [r, J] = residual(x)

    [r, J ] = FiniteDiff(@residual_noJ, x);
    
end

function r = noiseNorm (x)
    scaleMat = diag([x(7) x(7) x(7) 1 1 1]);
    r = scaleMat*T2Cart_ros(Cart2T_ros(x(1:6))); %propagate errors in the 6x1 space through T-space according to the parameter
end

function r = residual_noJ( x )
    global ctx;
    ctx.x = x;
    n = size(ctx.traj_c1,2);
    Tab = Cart2T_ros(x(1:6)); 

    %Get the Jacobian at this value of X 
    %as a 6x7 to capture differences in scaling...
    
    %[res, Jc1c2] = FiniteDiff(@noiseNorm, x );
    
    %Compute the initial offset that has to be taken out at the end
    %Without this, the transform Tab double-counts the rotations
    
    T_rpy = Cart2T_ros([0; 0; 0; x(4:6)]);
    
    r = [];
    for ii = 1:n

        %Using just subtraction
        
        total_res = abs(ctx.traj_c1(1:6,ii) - T2Cart_ros(Tab*Cart2T_ros([ctx.traj_c2(1:3,ii).*x(7);ctx.traj_c2(4:6,ii)])*inv(T_rpy)));
        %Fold in the scale matrix: all translation terms get scaled,
        %rotations pass through

        %S = chol(ctx.R1 + Jc1c2*blkdiag(ctx.R2,1)*Jc1c2'); %propagate error via Cholesky decomp
        %r = [r; inv(S)*total_res];
        
        %Compose the residual using simple subtraction on (x,y,z)
        %total_res = abs(ctx.traj_c1(1:3,ii) - Tab(1:3,:)*[ctx.traj_c2(1:3,ii).*x(7);1]);
        %s_xyz = S(1:3,1:3);
        r = [r; total_res];
    end
    plot_system( ctx );
    %waitforbuttonpress;
end

function plot_system( ctx )
    clf;
    hold on; axis vis3d; grid on; view(3);
    n = size(ctx.traj_c1,2);
    T = Cart2T_ros(ctx.x);

    
    xyz = ctx.traj_c1(1:3,:);
    plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'b-', 'LineWidth', 2 );

    xyz = T(1:3,:)*[ctx.traj_c2(1:3,:)*ctx.x(7); ones(1,n)]; %scale applied to xyz

    h = plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'r-', 'LineWidth', 2 );

    legend('Master Pose', 'Slave Pose');

    for ii = 1:n
       xyz = [ctx.traj_c1(1:3,ii)'; [T(1:3,:)*[ctx.traj_c2(1:3,ii).*ctx.x(7);1]]'];
       line( xyz(:,1),xyz(:,2),xyz(:,3), 'LineStyle',':', 'Color', 'c' );
       if (mod(ii,10) == 0)
           %plot the angular data as well
            %plot_ros_cf(ctx.traj_c1(1:6,ii),1);
  
            %Take out the initial angular offset that represents the
            %transform we're trying to find
            
            T_rpy = Cart2T_ros([0; 0; 0; ctx.x(4:6)]);
            %plot_ros_cf(T2Cart_ros(T*Cart2T_ros([ctx.traj_c2(1:3,ii).*ctx.x(7); ctx.traj_c2(4:6,ii)])*inv(T_rpy)),1);
       end
    end
    axis tight;
    drawnow();
end
