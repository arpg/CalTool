function SetupAeroView()

    hold on;
    axis vis3d;
    set(gcf, 'menubar', 'none');
    cameratoolbar('Show');
    cameratoolbar('setcoordsys','z');
    set(gca,'CameraUpVector', [0 0 -1] );
    set( gca, 'YDir', 'reverse' );
    grid on;

    xlabel('x (forward)');
    ylabel('y (right)');
    zlabel('z (down)');
end