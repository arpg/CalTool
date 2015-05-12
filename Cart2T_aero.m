function hpose = Cart2T_aero( epose )

	hpose(1:3,4) = epose( 1:3 );
	hpose(1:3,1:3) = Cart2R_aero( epose(4:6) );
	hpose( 4,: ) = [ 0 0 0 1];