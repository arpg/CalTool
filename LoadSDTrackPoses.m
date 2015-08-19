function [ poses ] = LoadSDTrackPoses(camfile)

    p = load( camfile );
    poses = zeros(6,size(p,1));
    for ii = 1:size(p,1)
        singlePose = p(ii,1:6);
        poses(:,ii) = singlePose;
    end
    
    %SDTrack does not produce vision convention frames
    %Appears to be robotics frame (x is fwd, y is left, z is up)
    
end

