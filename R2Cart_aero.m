%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert a 3x3 rotation matrix to a 3x1 Euler angle.
%
% Robotics Convention:
%  p = roll  about x axis, since +x is forward
%  q = pitch about y axis, since +y is right
%  r = yaw   about x axis, since +z is down
%
function euler = R2Cart_aero (R )

   if( isnumeric(R) && abs(det(R)-1) > 1e-9 )
        det(R)
        warning('ERROR: R2Cart_aero(R) -- det(R) ~= 1')
   end

   % Recall:
   % R = [ cr*cq, -sr*cp+cr*sq*sp,  sr*sp+cr*sq*cp ;...
   %       sr*cq,  cr*cp+sr*sq*sp, -cr*sp+sr*sq*cp ;...
   %         -sq,           cq*sp,           cq*cp ];
   %


   if isnumeric(R)
       euler(1,1) = atan2( R(3,2), R(3,3) );   % roll, psi or p
       euler(2,1) = -asin( R(3,1) );           % pitch, theta or q
       euler(3,1) = atan2( R(2,1), R(1,1) );   % yaw, phi or r
   else
       euler(1,1) = atan( R(3,2) / R(3,3) );   % roll, psi or p
       euler(2,1) = -asin( R(3,1) );           % pitch, theta or q
       euler(3,1) = atan( R(2,1) / R(1,1) );   % yaw, phi or r
   end
