function x = T2Cart_vis( T )
    x(1:3,1) = T(1:3,4);
    x(4:6,1) = R2Cart_vis(T(1:3,1:3));
end

