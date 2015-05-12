function DrawBoundingBox( bb )
    hold on;
    plot3( [bb.max_x; bb.min_x], [bb.max_y; bb.max_y], [bb.max_z; bb.max_z], 'r' );
    plot3( [bb.max_x; bb.min_x], [bb.max_y; bb.max_y], [bb.min_z; bb.min_z], 'r' );
    plot3( [bb.max_x; bb.min_x], [bb.min_y; bb.min_y], [bb.max_z; bb.max_z], 'r' );
    plot3( [bb.max_x; bb.min_x], [bb.min_y; bb.min_y], [bb.min_z; bb.min_z], 'r' );
    
    plot3( [bb.max_x; bb.max_x], [bb.max_y; bb.max_y], [bb.max_z; bb.min_z], 'b' );
    plot3( [bb.max_x; bb.max_x], [bb.min_y; bb.min_y], [bb.max_z; bb.min_z], 'b' );
    plot3( [bb.min_x; bb.min_x], [bb.max_y; bb.max_y], [bb.max_z; bb.min_z], 'b' );
    plot3( [bb.min_x; bb.min_x], [bb.min_y; bb.min_y], [bb.max_z; bb.min_z], 'b' );    
    
    plot3( [bb.max_x; bb.max_x], [bb.max_y; bb.min_y], [bb.max_z; bb.max_z], 'g' );
    plot3( [bb.max_x; bb.max_x], [bb.max_y; bb.min_y], [bb.min_z; bb.min_z], 'g' );
    plot3( [bb.min_x; bb.min_x], [bb.max_y; bb.min_y], [bb.max_z; bb.max_z], 'g' );
    plot3( [bb.min_x; bb.min_x], [bb.max_y; bb.min_y], [bb.min_z; bb.min_z], 'g' );

end

    