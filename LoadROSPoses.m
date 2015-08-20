function [ poses ] = LoadROSPoses(camfile)

    p = load( camfile );
    poses = zeros(6,size(p,1));
    for ii = 1:size(p,1)
        singleXYZ= p(ii,5:7);
        singleQuat = p(ii,8:11);
       [roll pitch yaw] = quat2angle(singleQuat);
       

        %Already in ROS frame
        pos = singleXYZ';
        poses(:,ii) = [pos; 0;0;yaw];
    end
end

