function p = ToOrigin_aero( pin )
    Tcw = inv(Cart2T_aero(pin(:,1)));
    for ii = 1:size(pin,2)
        Twci = Cart2T_aero(pin(:,ii));
        Tcci = Tcw*Twci;
        p(:,ii) = T2Cart_aero(Tcci);
    end
end