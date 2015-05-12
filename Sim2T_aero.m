function T = Sim2T_aero( s )
    T = Cart2T( s(1:6) );
    T(1:3,1:3) = T(1:3,1:3)*s(7);
end
