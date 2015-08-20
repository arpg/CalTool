
center_cam = LoadSDTrackPoses('~/datasets/extracted/center.csv');
left_cam = LoadSDTrackPoses('~/datasets/extracted/left.csv');
left_spline = makeSplinePath(left_cam, 10, 900);
center_spline = makeSplinePath(center_cam, 10, 900);