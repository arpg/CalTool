function p = ToOrigin_vis( pin )
    Tcw = inv(Cart2T_vis(pin(:,1)));
    for ii = 1:size(pin,2)
        Twci = Cart2T_vis(pin(:,ii));
        Tcci = Tcw*Twci;
        p(:,ii) = T2Cart_vis(Tcci);
    end
end