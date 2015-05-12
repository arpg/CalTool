function R = Cart2R_vis( pqr )

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

    % roll about z='forward', so move 'right' and 'down'
    Rroll = [ cp -sp  0;...
              sp  cp  0;...
               0   0  1];

    % pitch about x='right', so move 'forward' and 'down'
    Rpitch = [ 1   0   0;...
               0  cq -sq;...
               0  sq  cq];

    % yaw about y='down', so move 'forward' and 'right'
    Ryaw = [ cr  0  sr;...
              0  1   0;...
            -sr  0  cr];

	R = Ryaw*Rpitch*Rroll;
end

