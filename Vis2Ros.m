function ros_data = Vis2Ros( vis_data )
    for ii = 1:size(vis_data,2)
        ros_data(:,ii) = Vis2RosPointOrPose( vis_data(:,ii) );
    end
end

function ros_data = Vis2RosPointOrPose( vis_data )
    RDF_ros_from_vis = [      0     0     1     0;...
                             -1     0     0     0;...
                              0    -1     0     0;...
                              0     0     0     1];
	if( numel(vis_data) == 3 ) % just a point
        ros_data = RDF_ros_from_vis(1:3,:)*[vis_data;1];
    else
        Twp_vis = Cart2T_ros( vis_data );
        Twp_ros = RDF_ros_from_vis*Twp_vis;
        ros_data = [ Twp_ros(1:3,4); vis_data(4:6) ];
    end
end
