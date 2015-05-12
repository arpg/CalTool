
function R = Cart2R_ros( pqr )

    if isnumeric(pqr)
        p = pqr(1);
        q = pqr(2);
        r = pqr(3);
    else
        syms p q r;
    end

    cp = cos(p);
    sp = sin(p);
    cq = cos(q);
    sq = sin(q);
    cr = cos(r);
    sr = sin(r);

    % roll about x (with z up)
    Rroll = [ 1  0   0;...
              0 cp -sp;...
              0 sp  cp];

    % pitch about y (with z up)
    Rpitch = [ cq  0  sq;...
                0  1   0;...
              -sq  0  cq];

    % yaw about z (with z up)
    Ryaw = [ cr -sr  0;...
             sr  cr  0;...
              0   0  1];

	R = Ryaw*Rpitch*Rroll;
end

