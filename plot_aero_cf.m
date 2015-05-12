%
% h = plot_cf( pose, scale ) 
%	
%   Draw an origin indicator at 'pose'.  Pose is represented by a six vector,
%   <x,y,z,r,p,q>, composed of a 3D point, <x,y,z>, and an Euler
%   roll-pitch-yaw angle <r,p,q>. Angles are positive counterclockwise. The
%   orientation r=0,p=0,q=0 is +x forward, +y right and +z down. This is a
%   standard aeronautical right-handed convention.
%
%   The optional scale parameter scales the graphics object. 
%
function h = plot_aero_cf( p, scale )
    if( nargin == 1 )
		scale = 1;
	end

	R = scale*Cart2R_aero( p(4:6) );

    f = R(:,1)+p(1:3);
    r = R(:,2)+p(1:3);
    d = R(:,3)+p(1:3);

    h(1) = line( [p(1);f(1)], [p(2),f(2)], [p(3);f(3)], 'Color', 'r' );
    h(2) = line( [p(1);r(1)], [p(2),r(2)], [p(3);r(3)], 'Color', 'g' );
    h(3) = line( [p(1);d(1)], [p(2),d(2)], [p(3);d(3)], 'Color', 'b' );
end