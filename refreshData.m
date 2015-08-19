
center_cam = LoadSDTrackPoses('~/datasets/extracted/center.csv');
left_cam = LoadSDTrackPoses('~/datasets/extracted/left.csv');
left_spline = makeSplinePath(left_cam, 100, 900);
center_spline = makeSplinePath(center_cam, 100, 900);