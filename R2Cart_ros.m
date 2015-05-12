function pqr = R2Cart_ros( R )
% Recall R = 
% [ cos(q)*cos(r), cos(r)*sin(p)*sin(q) - cos(p)*sin(r), sin(p)*sin(r) + cos(p)*cos(r)*sin(q)]
% [ cos(q)*sin(r), cos(p)*cos(r) + sin(p)*sin(q)*sin(r), cos(p)*sin(q)*sin(r) - cos(r)*sin(p)]
%        -sin(q),                        cos(q)*sin(p),                        cos(p)*cos(q)]
   if isnumeric(R)
       pqr(1,1) = atan2( R(3,2), R(3,3) );   % roll
       pqr(2,1) = -asin( R(3,1) );           % pitch
       pqr(3,1) = atan2( R(2,1), R(1,1) );   % yaw
   else
       pqr(1,1) = atan( R(3,2) / R(3,3) );   % roll
       pqr(2,1) = -asin( R(3,1) );           % pitch
       pqr(3,1) = atan( R(2,1) / R(1,1) );   % yaw
   end   
end
