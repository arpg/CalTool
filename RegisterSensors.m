function RegisterSensors( camposes, armposes )
    global ctx;
    ctx = [];
    ctx.h = [];
    ctx.time_offset = 6;

    ctx.x = [zeros(12,1)];
    ctx.x = [ 0.0134    0.0308    0.4876   -1.5383   -0.0259    1.8833    0.0473    0.0267    0.5575   -1.5322   -0.0218    1.8872;]';
    ctx.x = [-0.0403    0.0239   -0.7617   -1.5383   -0.0259    1.8833   -0.0040    0.0313   -0.7478   -4.6165   -0.1943    4.7191  1]';
    ctx.x = [-0.0623    0.0435   -0.1898   -1.5383   -0.0259    1.8833   -0.0558    0.0468   -0.1918   -4.6164   -0.1909    4.7193    1.5320]';
    ctx.x = [-0.0758    0.0513   -0.4935   -1.5383   -0.0259    1.8833   -0.0769    0.0500   -0.4905   -4.6977   -0.2009    4.7405    1.5908]';
    ctx.x = [-0.0675    0.0513   -0.0655   -1.5383   -0.0259    1.8833   -0.0468    0.3405   -1.3471   -6.2689   -9.3521    2.9650    1.5943]';
    ctx.x = [-0.0711    0.0540    0.2328   -1.5383   -0.0259    1.8833   -0.0549    0.3364   -1.0546   -6.2841   -9.3499    2.9657    1.6106]';
    ctx.orig_traj_c1 = armposes;
    ctx.orig_traj_c2 = camposes;

    ApplyUpdate(ctx.x);
    PlotSystem();

    if 1
        DoTimeRegistration();
        return;
    end

    
    opts = optimset('Display', 'Iter');
    ctx.x = lsqnonlin( @residual, ctx.x, [], [], opts )
%    ctx.x = fminsearch( @squared_residual, ctx.x, opts );
    x = ctx.x'
end

function ApplyUpdate(x)
    global ctx;
    ctx.x = x;
    
    % shift entire trajectory (to sync time)
    if( ctx.time_offset >= 0)
        ctx.traj_c1 = ctx.orig_traj_c1(:,ctx.time_offset+1:end);
        ctx.traj_c2 = ctx.orig_traj_c2(:,1:size(ctx.traj_c1,2));
    else
        ctx.traj_c2 = ctx.orig_traj_c1(:,-ctx.time_offset:end);
        ctx.traj_c1 = ctx.orig_traj_c2(:,1:size(ctx.traj_c2,2));
    end

    ctx.Tc1c2 = Cart2T_aero(x(1:6));  % Tc1c2 NEVER includes scale
    if numel(x) == 6
         ctx.Tw1w2 = Cart2T_aero(x(1:6));
    elseif numel(x) == 7
         ctx.Tw1w2 = Sim2T_aero(x(1:7)); % Tw1w2 can include scale
    elseif numel(x) == 12
        ctx.Tw1w2 = Cart2T_aero(x(7:12));
    elseif numel(x) == 13
        ctx.Tw1w2 = Sim2T_aero(x(7:13)); % Tw1w2 can include scale
    else
        error('ApplyUpdate');
    end
%    fprintf( 'Tc1c2 = %f, %f, %f, %f, %f, %f\n', ctx.x(1:6)' );
%    fprintf( 'Tw1w2 = %f, %f, %f, %f, %f, %f\n', ctx.x(7:12)' );

    % generate "pedicted" trajectory 2 in frame 1 based on sensor pose
    % (Tc1c2 does not include scale -- that's Tw1w2's job).
    n = size(ctx.traj_c1,2);
    for ii = 1:n
        Tw1c2 = Cart2T_aero(ctx.traj_c1(:,ii))*ctx.Tc1c2;
        ctx.predicted_traj_c2(:,ii) = T2Cart_aero(Tw1c2);
    end
    % ... and "transformed" trajectory 2 in frame 1 based on frame
    % registration (Tw1w2 may include scale, so we normalize after applying
    % the transform).
    for ii = 1:n
        Tw1c2 = ctx.Tw1w2*Cart2T_aero(ctx.traj_c2(:,ii));
        ctx.transformed_traj_c2(:,ii) = T2Cart_aero( normalize(Tw1c2) );
    end
end

function c = squared_residual( x )
    r = residual(x);
    c = r.'*r;
end

function r = residual( x )
    global ctx;
    n = size(ctx.traj_c1,2);

    ApplyUpdate(x);
    PlotSystem();

    r = [];
    for ii = 1:n-1
        ab   = Cart2T_aero(ctx.transformed_traj_c2(:,ii));
        ab_p = Cart2T_aero(ctx.predicted_traj_c2(:,ii));
%        r = [r;  T2Cart_aero( normalize(ab*inv(ab_p))) ];
       r = [r; ctx.transformed_traj_c2(1:3,ii)-ctx.predicted_traj_c2(1:3,ii)];
%        r = [r; ctx.traj_c1(1:3,ii) - T(1:3,:)*[ctx.traj_c2(1:3,ii);1]];

%        Twai = Cart2T_aero( ctx.traj_c1(:,ii) );
%        Twbi = Cart2T_aero( ctx.traj_c2(:,ii) );
%        Twaj = Cart2T_aero( ctx.traj_c1(:,ii+1) );
%        Twbj = Cart2T_aero( ctx.traj_c2(:,ii+1) );
%        Taiaj = inv(Twai)*Twaj;
%        Tbibj = inv(Twbi)*Twbj;
%        Terror = inv(Tbibj)*Twbi*inv(ctx.Tc1c2)*Taiaj*ctx.Tc1c2;
%        r = [r; T2Cart( normalize(Terror)) ];
%        r = [ r; Terror(1:3,4)];
    end
    r'*r;
end

function PlotSystem()
    global ctx;
    n = size(ctx.traj_c1,2);
    
    if ~isempty(ctx.h) 
        delete( ctx.h );
        ctx.h = [];
    end

%    h1 = plot_aero_path(ctx.traj_c1,0.01);
%    h2 = plot_aero_path(ctx.predicted_traj_c2,0.01);
%    h3 = plot_aero_path(ctx.transformed_traj_c2,0.01);
%    ctx.h = [ h2 h3];

    xyz = ctx.predicted_traj_c2(1:3,:);
    ctx.h(end+1) = plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'r--', 'LineWidth', 2 );
    xyz = ctx.transformed_traj_c2(1:3,:);
    ctx.h(end+1) = plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'g--', 'LineWidth', 2 );
 %   xyz = ctx.traj_c1(1:3,:);
 %   ctx.h(end+1) = plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'b-', 'LineWidth', 2 );
    
  %  legend('camera predictied from arm pose','transformed camera traj','arm traj');

    for ii = 1:n
%       xyz = [ctx.traj_c1(1:3,ii)'; ctx.predicted_traj_c2(1:3,ii)'];
%       ctx.h(end+1) = line( xyz(:,1),xyz(:,2),xyz(:,3), 'LineStyle',':', 'Color', 'c' );
       
       xyz = [ctx.transformed_traj_c2(1:3,ii)'; ctx.predicted_traj_c2(1:3,ii)'];
       ctx.h(end+1) = line( xyz(:,1),xyz(:,2),xyz(:,3), 'LineStyle', ':', 'Color', 'r' );
    end
 %   axis tight;
    drawnow();
end


function T = normalize( T )
    T(1:3,1:3) = T(1:3,1:3) / norm(T(1:3,1));
end


function DoTimeRegistration()
    global ctx;
    set( gcf, 'WindowKeyPressFcn', @(h_obj,evt)[] );
    view(3);
    while 1
        k = double(get( gcf, 'currentcharacter' ));
        if k == 'a'
            ctx.time_offset = ctx.time_offset - 1;
            fprintf('Trajectories are offset by %d indices\', ctx.time_offset );
        elseif k == 'd'
            ctx.time_offset = ctx.time_offset + 1;
            fprintf('Trajectories are offset by %d indices\', ctx.time_offset );
        end
        set(gcf,'currentch',char(1)); % reset CurrentCharacter        
        ApplyUpdate(ctx.x);
        PlotSystem();
    end
end
