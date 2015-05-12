% 
% poses.csv is produced using the following commandline:
% 
% GLOG_logtostderr=1  ./vicalib  -cam file://./image* -grid_preset=letter
% -exit_vicalib_on_finish=false -save_poses=true -model_files=cameras.xml
% -calibrate_intrinsics=false
% 
% where cameras.xml is the already calibrated camera model.
% 

function [CamPoses,CamTimes,ArmPoses,ArmTimes] = ReadArmAndCamPoses( srcDir )
    p = load( strcat(srcDir,'/poses.csv') );
    CamPoses = zeros(6,size(p,1));
    for ii = 1:size(p,1)
       Ttc = [p(ii,1:4); p(ii,5:8); p(ii,9:12); 0 0 0 1];
       CamPoses(:,ii) = T2Cart_vis(inv(Ttc));
    end
    CamPoses = Vis2Aero(CamPoses);

    frameList = [1:1:size(CamPoses,2)];
    %Go through the srcDir, find all pgms, read the header and extract the 6dof
    %pose from the header lines
    srcDir_aug = sprintf('%s/*.pgm', srcDir);
    listing = dir(srcDir_aug);
    
    ArmPoses = [];
    CamTimes = [];
    ArmTimes = [];
    for i=1:length(frameList)
        %open  the pgm, extract the arm pose from the comments
        fileName = sprintf('%s/%s', srcDir, listing(frameList(i)).name);
        theFile = fopen(fileName);
        
        %First line: magic number
        tline = fgetl(theFile);
        
        %Second: size
        tline = fgetl(theFile);
        %Third: Max val
        tline = fgetl(theFile);
        
        %Fourth: Comment incl geometry
        tline = fgetl(theFile);
        
        %Strip the leading comment
        tline = tline(:,2:end);
        geom = sscanf(tline, '%d, %f, %f, %f, %f, %f, %f, %f, %f');
        camtime = geom(2);
        CamTimes = [CamTimes camtime];
        ArmPoses = [ArmPoses geom(3:end)];
        
        %Fifth: whitespce
        tline = fgetl(theFile);
        
        %Sixth: arm time
        tline = fgetl(theFile);
        tline = tline(:,2:end);
        armtime = sscanf(tline, '%f');
        ArmTimes = [ArmTimes armtime];
        fclose(theFile);
    end
    
    ArmPoses = Ros2Aero(ArmPoses);
end
