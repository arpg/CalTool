function h = plot_ros_cf( pose, scale )
  if( nargin == 1 )
    scale = 1;
  end
  R = scale*Cart2R_ros( pose(4:6) );
  h(1) = line( [0; R(1,1)]+pose(1), [0; R(2,1)]+pose(2), [0; R(3,1)]+pose(3), 'Color', 'r' );
  h(2) = line( [0; R(1,2)]+pose(1), [0; R(2,2)]+pose(2), [0; R(3,2)]+pose(3), 'Color', 'g' );
  h(3) = line( [0; R(1,3)]+pose(1), [0; R(2,3)]+pose(2), [0; R(3,3)]+pose(3), 'Color', 'b' );
end