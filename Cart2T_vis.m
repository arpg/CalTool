function T = Cart2T_vis( x )
    T = [ Cart2R_vis(x(4:6)) x(1:3);...
          0      0       0       1  ];
end