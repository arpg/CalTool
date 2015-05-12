% draw a robot trajectory
% rpath is a 6xN vector of 6D poses
function h = plot_vis_path( rpath, scale )

    if( nargin == 1 )
        scale = 1;
    end

    SetupVisionView();

    h = [];
    for ii = 1:size(rpath,2)
        h = [h plot_vis_cf( rpath(:,ii), scale )];
    end

    h = [h plot3( rpath(1,:), rpath(2,:), rpath(3,:), 'm-' ) ];
    h = [h plot3( rpath(1,1), rpath(2,1), rpath(3,1), 'm.' ) ];
    h = [h plot3( rpath(1,end), rpath(2,end), rpath(3,end), 'm.' ) ];
end