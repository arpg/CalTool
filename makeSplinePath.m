function [dest] = makeSplinePath(srcTraj, segments, poseCount)

%Break Down the source trajectory
x = srcTraj(1,:);
y = srcTraj(2,:);
z = srcTraj(3,:);

%Abs for rotations so spline works
p = unwrap(srcTraj(4,:));
q = unwrap(srcTraj(5,:));
r = unwrap(srcTraj(6,:));

xx = 1:length(x);

%Make the PP things
pp1 = splinefit(xx,x,segments);
pp2 = splinefit(xx,y,segments);
pp3 = splinefit(xx,z,segments);
pp4 = splinefit(xx,p,segments);
pp5 = splinefit(xx,q,segments);
pp6 = splinefit(xx,r,segments);

breaks = linspace(0, length(xx), poseCount);

%Output the interpolated poses
dest = [ppval(pp1,breaks);
           ppval(pp2,breaks);
           ppval(pp3,breaks);
           ppval(pp4,breaks);
           ppval(pp5,breaks);
           ppval(pp6,breaks);];

end
