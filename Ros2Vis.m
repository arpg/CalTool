function vis_data = Ros2Vis( ros_data )
    for ii = 1:size(ros_data,2)
        vis_data(:,ii) = Ros2VisPointOrPose( ros_data(:,ii) );
    end
end

function vis_data = Ros2VisPointOrPose( ros_data )
    RDF_vis_from_ros = [  0 -1  0  0;...
                          0  0 -1  0;...
                          1  0  0  0;...
                          0  0  0  1];
	if( numel(ros_data) == 3 ) % just a point
        vis_data = RDF_vis_from_ros(1:3,:)*[ros_data;1];
    else
        Twp_ros = Cart2T_ros( ros_data );
        Twp_vis = RDF_vis_from_ros*Twp_ros;
        vis_data = [ Twp_vis(1:3,4); ros_data(4:6) ];
    end
end
