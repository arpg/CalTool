function [ poses ] = LoadSDTrackPoses(camfile)

    p = load( camfile );
    poses = zeros(6,size(p,1));
    for ii = 1:size(p,1)
        singlePose = p(ii,1:6);
        poses(:,ii) = singlePose;
    end
end

