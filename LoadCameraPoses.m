function poses = LoadCameraPoses( camfile )
    p = load( camfile );
    poses = zeros(6,size(p,1));
    for ii = 1:size(p,1)
       Ttc = [p(ii,1:4); p(ii,5:8); p(ii,9:12); 0 0 0 1];
       poses(:,ii) = T2Cart_vis(inv(Ttc));
    end
end
