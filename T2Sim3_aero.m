function s = T2Sim_aero( S )
    T = S;
    T(1:3,1:3) = S(1:3,1:3)./norm(S(1:3,1));
    s = T2Cart_aero( T );
end
