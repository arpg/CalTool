

function [poses] = readArmPoses(frameList, srcDir)
  %Go through the srcDir, find all pgms, read the header and extract the 6dof
  %pose from the header lines
  srcDir_aug = sprintf('%s/*.pgm', srcDir);
  listing = dir(srcDir_aug);

  poses = [];
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
      time = geom(2);
      poses = [poses geom(3:end)];
      
      %Fifth: whitespce
      tline = fgetl(theFile)
     
      %Sixth: arm time
      tline = fgetl(theFile);
      tline = tline(:,2:end);
      armtime = sscanf(tline, '%f');
      fclose(theFile);
  end
end
