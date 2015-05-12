function pqr = R2Cart_vis( R )
% Recall R = 

%[ cos(p)*cos(r) + sin(p)*sin(q)*sin(r), cos(p)*sin(q)*sin(r) - cos(r)*sin(p), cos(q)*sin(r)]
%[                        cos(q)*sin(p),                        cos(p)*cos(q),       -sin(q)]
%[ cos(r)*sin(p)*sin(q) - cos(p)*sin(r), sin(p)*sin(r) + cos(p)*cos(r)*sin(q), cos(q)*cos(r)]

  pqr(1,1) =  atan2( R(2,1), R(2,2) ); % roll
  pqr(2,1) = -asin( R(2,3) );            % pitch
  pqr(3,1) =  atan2( R(1,3), R(3,3) );   % yaw
end
