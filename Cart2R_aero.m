%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% R = eulerPQR_to_rotmat( attitude )
%  
% Construct rotation matrix using ZYX-fixed frame convention.
%
% Note that I use the Roll-Pitch-Yaw convention, where r is yaw, q is pitch
% and p is roll  i.e. attitude = (roll,pitch,yaw) = (p,q,r)
%
% Aircraft control and simulation By Brian L. Stevens, Frank L. Lewis says
% p,q,r are standard symbols for roll, pitch and yaw.
%
% This is rotation about the fixed frame. see eqn 2.20 of Modelling and
% Control of Robot Manipulators.
%
%  p, roll  about x axis, sinze +x is forward
%  q, pitch about y axis, since +y is right
%  r, yaw   about x axis, sinze +z is down
%
function R = eulerPQR_to_rotmat(attitude)

% WARNING this derviation uses "rpq" instead of the more standard "pqr" to
% represent roll,pitch,yaw.  The above has been fixed.

%    attitude = angle_wrap( attitude );

%  verify
%    syms p q r;
%    attitude = [p,q,r];

    cp = cos( attitude(1) );
    cq = cos( attitude(2) );
    cr = cos( attitude(3) );

    sp = sin( attitude(1) );
    sq = sin( attitude(2) );
    sr = sin( attitude(3) );

    % check roll, p:
    %     Rx(pi/2)*[0;1;0] = [0;   0; 1]
    %     Rx(pi/2)*[0;0;1] = [0;  -1; 0]
    Rx = [  1     0    0 ;...
            0    cp  -sp ; ...
            0    sp   cp];

    % check pitch, q:
    %     Ry(pi/2)*[1;0;0] = [ 0;  0; -1]
    %     Ry(pi/2)*[0;0;1] = [ 1;  0;  0]
    Ry = [  cq   0   sq  ; ...
             0   1    0  ; ...
           -sq   0   cq ];

    % check yaw, r:
    %     Rz(pi/2)*[1;0;0] = [ 0;  1; 0]
    %     Rz(pi/2)*[0;1;0] = [-1;  0; 0]
    Rz = [  cr   -sr    0; ...
            sr    cr    0; ...
             0     0    1];

    % ZYX order: roll, pitch, yaw
    R = Rz*Ry*Rx;

% same thing as:
%R = [ cos(r)*cos(q), -sin(r)*cos(p)+cos(r)*sin(q)*sin(p),  sin(r)*sin(p)+cos(r)*sin(q)*cos(p) ;...
%      sin(r)*cos(q),  cos(r)*cos(p)+sin(r)*sin(q)*sin(p), -cos(r)*sin(p)+sin(r)*sin(q)*cos(p) ;...
%            -sin(q),                       cos(q)*sin(p),                       cos(q)*cos(p) ];
%
% This is rotation about the fixed frame. see eqn 2.20 of Modelling and Control of Robot Manip.
%

