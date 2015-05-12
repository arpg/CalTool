function ros_data = Aero2Ros( aero_data )
    for ii = 1:size(aero_data,2)
        ros_data(:,ii) = Aero2RosPointOrPose( aero_data(:,ii) );
    end    
end

function ros_data = Aero2RosPointOrPose( aero_data )
    RDF_ros_from_aero = [ 1     0     0     0;...
                          0    -1     0     0;...
                          0     0    -1     0;...
                          0     0     0     1];
	if( numel(aero_data) == 3 ) % just a point
        ros_data = RDF_ros_from_aero(1:3,:)*[aero_data;1];
    else
        Twp_aero = Cart2T_aero( aero_data );
        Twp_ros = RDF_ros_from_aero*Twp_aero;
        ros_data = [ Twp_ros(1:3,4); aero_data(4:6) ];
    end
end
