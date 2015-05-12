function epose = T2Cart_aero( hpose )
	epose(1:3,1) = hpose(1:3,4);
	epose(4:6,1) = R2Cart_aero( hpose(1:3,1:3) );
