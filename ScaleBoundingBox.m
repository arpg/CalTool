
function bb = ScaleBoundingBox( bb, scale )

    bb.min_x = bb.min_x + bb.dx*scale/2;
    bb.max_x = bb.max_x - bb.dx*scale/2;

    bb.min_y = bb.min_y + bb.dy*scale/2;
    bb.max_y = bb.max_y - bb.dy*scale/2;
    
    bb.min_z = bb.min_z + bb.dz*scale/2;
    bb.max_z = bb.max_z - bb.dz*scale/2;
   
    bb.dx = bb.max_x - bb.min_x;
    bb.dy = bb.max_y - bb.min_y;
    bb.dz = bb.max_z - bb.min_z;
end