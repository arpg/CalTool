
function T = Cart2T_ros( x )
    T = [ Cart2R_ros( x(4:6) ) x(1:3);...
          0      0        0       1  ];
end