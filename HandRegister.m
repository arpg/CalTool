function HandRegister
    global ctx;
    ctx.h = [];
    close all;
    
    rawcamposes = LoadCameraPoses('cal_poses.csv'); % in vision frame
    idxs = [1:40:1450];
    camposes = Vis2Aero( rawcamposes(:,idxs) );
    rosposes = readArmPoses(idxs, '/Users/gsibley/Data/ArmCal/calibration_data');
    rosposes = ToOrigin_ros( rosposes );
    armposes = Ros2Aero( rosposes );
    camposes = ToOrigin_aero( camposes );
    armposes = ToOrigin_aero( armposes );

%     pose = [0;0;0; 0;pi/4;pi/4]
%     Ros2Aero(pose)
%     view(3); SetupRosView();
%     plot_ros_cf( zeros(6,1) );
%     plot_ros_cf( pose,0.5 );
% 
%     figure; view(3); SetupAeroView();
%     plot_aero_cf( zeros(6,1) );
%     plot_aero_cf( Ros2Aero(pose), 0.5 );
% 
%     return;
    
%     view(3);
%     plot_ros_path( rosposes, 0.01 );
%     
%     figure; view(3);
%     plot_aero_path( armposes, 0.01 );
% 
%     return;
%     
    set( gcf, 'WindowKeyPressFcn', @(h_obj,evt)[] );
    view(3);

    ctx.traj_c1 = armposes;
    ctx.traj_c2 = camposes;

    % set initial registration
    ctx.x = [zeros(6,1);1];
    
    Tc1c2 = Sim2T_aero( ctx.x );
    Tw1w2 = Tc1c2;

    ApplyUpdate( ctx.x );
    
    while 1
       dpose = GetCommand();
       ctx.x = scomp( ctx.x, [dpose;1] );
       ApplyUpdate( ctx.x );
       PlotResults();
%       pause(0.5);
    end
    
end

function PlotResults()
    global ctx;
    if ~isempty(ctx.h)
        delete(ctx.h);
        ctx.h = [];
    end
    SetupAeroView();
    
    h1 = plot_aero_path(ctx.traj_c1,0.01);
    h2 = plot_aero_path(ctx.predicted_traj_c2,0.01);
    h3 = plot_aero_path(ctx.transformed_traj_c2,0.01);
    ctx.h = [h1 h2 h3];

    xyz = ctx.predicted_traj_c2(1:3,:);
    ctx.h(end+1) = plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'r--', 'LineWidth', 2 );
    xyz = ctx.transformed_traj_c2(1:3,:);
    ctx.h(end+1) = plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'g--', 'LineWidth', 2 );
    xyz = ctx.traj_c1(1:3,:);
    ctx.h(end+1) = plot3( xyz(1,:), xyz(2,:), xyz(3,:), 'b-', 'LineWidth', 2 );
    legend('camera predictied from arm pose','transformed camera traj','arm traj');
    
    n = size( ctx.traj_c1,2);
    for ii = 1:n
        xyz = [ctx.traj_c1(1:3,ii)'; ctx.predicted_traj_c2(1:3,ii)'];
        ctx.h(end+1) = line( xyz(:,1),xyz(:,2),xyz(:,3), 'LineStyle',':', 'Color', 'c' );

        xyz = [ctx.transformed_traj_c2(1:3,ii)'; ctx.predicted_traj_c2(1:3,ii)'];
        ctx.h(end+1) = line( xyz(:,1),xyz(:,2),xyz(:,3), 'LineStyle', ':', 'Color', 'r' );
    end
    
    drawnow();
end



function ApplyUpdate( x )
    global ctx;
    Tw1w2 = Sim2T_aero( x );
    Tc1c2 = Tw1w2;
    ctx.x = x;
    n = size(ctx.traj_c2,2);
    for ii = 1:n
        s = T2Sim_aero(Tw1w2*Cart2T_aero(ctx.traj_c2(:,ii)));
        ctx.transformed_traj_c2(:,ii) = s(1:6,1);
    end
    for ii = 1:n
        s = T2Sim_aero(Cart2T_aero(ctx.traj_c1(:,ii))*Tc1c2);
        ctx.predicted_traj_c2(:,ii) = s(1:6,1);
    end
end

function dpose = GetCommand()
    dpose = zeros(6,1);
    k = double(get( gcf, 'currentcharacter' ));
    if isempty(k)
        return
    end
    switch k
        case 'a'
            dpose = [ 0;-0.01;0; 0;0;0];
        case 's'
            dpose = [ -0.01;0;0; 0;0;0];
        case 'd'
            dpose = [ 0;0.01;0; 0;0;0];
        case 'w'    
            dpose = [ 0.01;0;0; 0;0;0];
        case 'q'    
            dpose = [ 0;0;0.01; 0;0;0];
        case 'e'    
            dpose = [ 0;0;-0.01; 0;0;0];
        case 'A'
            dpose = [ 0;0;0; 0.1; 0; 0];
        case 'S'
            dpose = [ -0.01;0;0; 0;-0.1;0];
        case 'D'
            dpose = [ 0;0.01;0; -0.1;0;0];
        case 'W'    
            dpose = [ 0.01;0;0; 0;0.1;0];
        case 'Q'    
            dpose = [ 0;0;0.01; 0;0;0.1];
        case 'E'    
            dpose = [ 0;0;-0.01; 0;0;-0.1];
    end
    set(gcf,'currentch',char(1)); % reset CurrentCharacter
end

function xac = scomp( xab, xbc )
    Sab = Sim2T_aero(xab);
    Sbc = Sim2T_aero(xbc);
    Tac = Sab*Sbc;
    xac = T2Sim_aero( Tac );
end
