function aero_data = Vis2Aero( vis_data )
    for ii = 1:size(vis_data,2)
        aero_data(:,ii) = Vis2AeroPointOrPose( vis_data(:,ii) );
    end
end

function aero_data = Vis2AeroPointOrPose( vis_data )
    RDF_aero_from_vis = [      0     0     1     0;...
                              -1     0     0     0;...
                               0    -1     0     0;...
                               0     0     0     1];
	if( numel(vis_data) == 3 ) % just a point
        aero_data = RDF_aero_from_vis(1:3,:)*[vis_data;1];
    else
        Twp_vis = Cart2T_aero( vis_data );
        Twp_aero = RDF_aero_from_vis*Twp_vis;
        aero_data = [ Twp_aero(1:3,4); vis_data(4:6) ];
    end
end
