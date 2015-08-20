%Include noise parameters on the trajectory information - allow for
%zero-information trajectory data

%pan_dyna is in Dynamixel units, or 1/4096th of a circle (0.088 degree)
%tilt_dyna is in Dynamixel units, or 1/4096th of a circle (0.088 degree)

function RegisterTraj_Noise_Pan_Tilt( pg1, pg2, pan_dyna, tilt_dyna)
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

    %Pretend that the source trajectory has bad height data and only yaw
    %info (rotation about Z axis)
    %ctx.traj_c1 = [ctx.traj_c1(1:2,:); zeros(1,n); zeros(2, n); ctx.traj_c1(6,:) ];
    
    %Adjust the uncertainties to reflect:
    noData = 5;
    R2 = diag([r2_sigma2_trans, r2_sigma2_trans, noData, noData, noData, r2_sigma2_rot ]);

    
    %Include the sensor noises in the context
    ctx.R1 = R1;
    ctx.R2 = R2;
    
    ctx.iter=1;
    
    ctx.x = [0.1, 0.2, 0.3, 0.1, 0.2, 0.3, ...
             -0.1, -0.2, -0.3, 0.1, 0.2, 0.3, ...
             0.5]';    
 
pan_six = [ 0, 0, 0, 0,0, pan_dyna/4096*2*pi ]';

    ctx.Tpanbase_movable = Cart2T_ros(pan_six);
    ctx.Tmovable_tilt = Cart2T_ros([ 0, 0, 0, 0,tilt_dyna/4096*2*pi, 0 ]');
    
    
    plot_system( ctx );
    opts = optimset('Display', 'Iter', 'MaxFunEvals', 2000);
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
    
%     [res, Jc1c2] = FiniteDiff(@noiseNorm, ctx.x );
%     Runit = ctx.R2 + Jc1c2*blkdiag(ctx.R1,1)*Jc1c2'; 
%     
%     %Prepare a complete R block:
%     meas_rz = makeBlkDiag(Runit, n);

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

function r = noiseNorm (x, ctx)
    scaleFactor = x(13);
    scaleMat = diag([scaleFactor scaleFactor scaleFactor 1 1 1]);

    %propagate errors in the 6x1 space through T-space according to the parameter
    
    Tbase_panbase = Cart2T_ros(x(1:6)); 
    Ttilt_cam2 = Cart2T_ros(scaleMat*x(7:12));
    
    r = T2Cart_ros(Tbase_panbase*ctx.Tpanbase_movable*ctx.Tmovable_tilt*Ttilt_cam2);
end

%X is now a 13-vector, 6dof for Tbase_panbase, 6dof for Tpanmoveable_cam2,
%and a scale factor for the camera
%Tpanbase_movable is a given
function r = residual_noJ( x )
    global ctx;
    ctx.x = x;
    n = size(ctx.traj_c1,2);
    Tbase_panbase = Cart2T_ros(x(1:6)); 
    Ttilt_cam2 = Cart2T_ros(x(7:12));
    scaleFactor = x(13);
    totalT = Tbase_panbase*ctx.Tpanbase_movable*ctx.Tmovable_tilt*Ttilt_cam2;
    
    T_rpy_base = Cart2T_ros([0; 0; 0; ctx.x(4:6)]);
    fixed_pan = T2Cart_ros(ctx.Tpanbase_movable);

    T_rpy_pan = Cart2T_ros([0; 0; 0; fixed_pan(4:6)]);
    
    fixed_tilt = T2Cart_ros(ctx.Tmovable_tilt);
    T_rpy_tilt = Cart2T_ros([0; 0; 0; fixed_tilt(4:6)]);
    
    T_rpy_bar = Cart2T_ros([0; 0; 0; ctx.x(10:12)]);   
    T_unwrap = inv(T_rpy_bar)*inv(T_rpy_tilt)*inv(T_rpy_pan)*inv(T_rpy_base);
    T_unwrap = eye(4);
    
    %Get the Jacobian at this value of X 
    %as a 6x13 to capture differences in scaling...
    
    [res, Jc1c2] = FiniteDiff(@noiseNorm, x , ctx);
    
    %Compute the initial offset that has to be taken out at the end
    %Without this, the initial transform double-counts the rotations
    
    T_rpy = Cart2T_ros([0; 0; 0; x(4:6)]);
    %*inv(T_rpy)
    r = [];
    for ii = 1:n

        %Using just subtraction
        
        %total_res = abs(ctx.traj_c1(1:6,ii) - T2Cart_ros(totalT*Cart2T_ros([ctx.traj_c2(1:3,ii).*scaleFactor;ctx.traj_c2(4:6,ii)])*T_unwrap));
        %Fold in the scale matrix: all translation terms get scaled,
        %rotations pass through

        
        %Compose the residual using simple subtraction on (x,y,z)
        total_res = abs([ctx.traj_c1(1:3,ii); 1] - totalT*[ctx.traj_c2(1:3,ii).*scaleFactor;1]);
        %s_xyz = S(1:3,1:3);

        %if (ii == n)
        %    disp('At end');
        %end
        %S = chol(ctx.R1 + Jc1c2*Jc1c2'*ctx.R2); %propagate error via Cholesky decomp
        %total_res = inv(S)*[total_res' 0 0]';
        
        r = [r; total_res]; 
    end
    ctx.iter =  ctx.iter + 1;
    %if mod(ctx.iter,10)
        plot_system( ctx );
        %waitforbuttonpress;
    %end
    
end

function plot_system( ctx )
    clf;
    hold on; axis vis3d; grid on; view(3);
    n = size(ctx.traj_c1,2);
    
    Tbase_panbase = Cart2T_ros(ctx.x(1:6)); 
    Ttilt_cam2 = Cart2T_ros(ctx.x(7:12));
    scaleFactor = ctx.x(13);
    totalT = Tbase_panbase*ctx.Tpanbase_movable*ctx.Tmovable_tilt*Ttilt_cam2;
    
    
    xyz = ctx.traj_c1(1:3,:);
    plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'b-', 'LineWidth', 2 );

    xyz = totalT*[ctx.traj_c2(1:3,:)*scaleFactor; ones(1,n)]; %scale applied to xyz

    h = plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'r-', 'LineWidth', 2 );

    legend('Master Pose', 'Slave Pose');

    for ii = 1:n
        slave_coord =  [totalT*[ctx.traj_c2(1:3,ii).*scaleFactor;1]]';
       xyz = [ctx.traj_c1(1:3,ii)'; slave_coord(1:3)];
       line( xyz(:,1),xyz(:,2),xyz(:,3), 'LineStyle',':', 'Color', 'c' );
       if (mod(ii,10) == 0)
           %plot the angular data as well
            plot_ros_cf(ctx.traj_c1(1:6,ii),1);
  
            %Take out the initial angular offset that represents the
            %transform we're trying to find
            %*inv(T_rpy)
            T_rpy_base = Cart2T_ros([0; 0; 0; ctx.x(4:6)]);
            fixed_pan = T2Cart_ros(ctx.Tpanbase_movable);
            fixed_tilt = T2Cart_ros(ctx.Tmovable_tilt);
            T_rpy_tilt = Cart2T_ros([0; 0; 0; fixed_tilt(4:6)]);
            T_rpy_pan = Cart2T_ros([0; 0; 0; fixed_pan(4:6)]);
            T_rpy_bar = Cart2T_ros([0; 0; 0; ctx.x(10:12)]);   
            T_unwrap = inv(T_rpy_bar)*inv(T_rpy_tilt)*inv(T_rpy_pan)*inv(T_rpy_base);
            T_unwrap = eye(4);
    
            

            plot_ros_cf(T2Cart_ros(totalT*Cart2T_ros([ctx.traj_c2(1:3,ii).*scaleFactor; ctx.traj_c2(4:6,ii)])*T_unwrap),1);
       end
    end
    axis tight;
    drawnow();
end
