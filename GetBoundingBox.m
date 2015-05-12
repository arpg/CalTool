function bb = GetBoundingBox( pts )
    bb.max_x = max(pts(1,:));
    bb.min_x = min(pts(1,:));
	bb.max_y = max(pts(2,:));
    bb.min_y = min(pts(2,:));
    bb.max_z = max(pts(3,:));
    bb.min_z = min(pts(3,:));
    bb.dx = bb.max_x - bb.min_x;
    bb.dy = bb.max_y - bb.min_y;
    bb.dz = bb.max_z - bb.min_z;
end
