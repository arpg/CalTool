function p = ToOrigin_ros( pin )
    Tcw = inv(Cart2T_ros(pin(:,1)));
    for ii = 1:size(pin,2)
        Twci = Cart2T_ros(pin(:,ii));
        Tcci = Tcw*Twci;
        p(:,ii) = T2Cart_ros(Tcci);
    end
end