close all;

[camposes,ctimes,armposes,atimes] = ReadArmAndCamPoses('/Users/gsibley/Data/ArmCal/calibration_synched');

bEvalTimeData = 0;
if bEvalTimeData
    [m,b,r] = fitline( ctimes );
    hold on;
    plot( ctimes, 'b.', 'MarkerSize', 1 );
    plot( m*x + b, 'r' );
    figure; hist(r,100);
    figure; hist(atimes-ctimes,20);
    return;
end

%rawcamposes = LoadCameraPoses('cal_poses.csv'); % in vision frame
%rawcamposes = LoadCameraPoses('/Users/gsibley/Data/ArmCal/calibration_synched/poses.csv'); % in vision frame
%idxs = [1:1:size(rawcamposes,2)];
%camposes = Vis2Aero( rawcamposes(:,idxs) );
%rosposes = readArmPoses(idxs, '/Users/gsibley/Data/ArmCal/calibration_data');
%rosposes = readArmPoses(idxs, '/Users/gsibley/Data/ArmCal/calibration_synched');
%rosposes = ToOrigin_ros( rosposes );
%armposes = Ros2Aero( rosposes );
%camposes = ToOrigin_aero( camposes );
%armposes = ToOrigin_aero( armposes );

%figure; hold on; view(3); SetupAeroView();
%plot_aero_cf( zeros(6,1) );
%plot_aero_path( armposes, 0.01 );

%figure; hold on; view(3); SetupAeroView();
%plot_aero_cf( zeros(6,1), 0.5 );
%plot_aero_path( camposes, 0.01 );

%return;

%scale_estiamte = (bbc.dx + bbc.dy + bbc.dz ) / (bba.dx + bba.dy + bba.dz )

view(3);
SetupAeroView();
%DrawBoundingBox( bbc );
%DrawBoundingBox( bba );

%plot_vision_path( camposes, 0.01 );
%axis tight;

%RegisterTrajectories( camposes, armposes );
RegisterSensors( camposes, armposes );

