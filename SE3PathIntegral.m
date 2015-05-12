function d = SE3PathIntegral( p, turnweight )
    d = zeros(1,size(p,2));
    for ii = 2:size(p,2)
        %d = [ d, d+norm( p(1:3,ii)-p(1:3,ii-1) ) ];
        d(ii) = d(ii-1) + norm( p(1:3,ii)-p(1:3,ii-1) )...
            + turnweight*norm( p(4:6,ii)-p(4:6,ii-1) );
    end
end
