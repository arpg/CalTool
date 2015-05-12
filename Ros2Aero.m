function aero_data = Ros2Aero( ros_data )
    for ii = 1:size(ros_data,2)
        aero_data(:,ii) = Ros2AeroPointOrPose( ros_data(:,ii) );
    end
end

function aero_data = Ros2AeroPointOrPose( ros_data )
    RDF_aero_from_ros = [ 1  0  0  0;...
                          0 -1  0  0;...
                          0  0 -1  0;...
                          0  0  0  1];
	if( numel(ros_data) == 3 ) % just a point
        aero_data = RDF_aero_from_ros(1:3,:)*[ros_data;1];
    else
        Twp_ros = Cart2T_ros( ros_data );
        Twp_aero = RDF_aero_from_ros*Twp_ros;
        pqr = RDF_aero_from_ros(1:3,1:3)*ros_data(4:6);
        aero_data = [Twp_aero(1:3,4); pqr ];
    end
end