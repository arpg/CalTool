function plotSpline(srcTraj, segments, poseCount)
%Close all other plots
close all;

splineTraj = makeSplinePath(srcTraj, segments, poseCount);
%Plot Truth vs Spline Estimate
figure(1);
subplot(1,2,1);
plot_vis_path(srcTraj, 0.2);
title('Truth Path');

subplot(1,2,2);
plot_vis_path(splineTraj, 0.2);
title('Spline Path');
end