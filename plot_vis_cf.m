function h = plot_vis_cf( p, scale )
  if( nargin == 1 )
    scale = 1;
  end
  R = scale*Cart2R_vis( p(4:6) );
  
  r = R(:,1)+p(1:3);
  d = R(:,2)+p(1:3);
  f = R(:,3)+p(1:3);

  h(1) = line( [p(1);f(1)], [p(2),f(2)], [p(3);f(3)], 'Color', 'b' );
  h(2) = line( [p(1);r(1)], [p(2),r(2)], [p(3);r(3)], 'Color', 'r' );
  h(3) = line( [p(1);d(1)], [p(2),d(2)], [p(3);d(3)], 'Color', 'g' );

end
