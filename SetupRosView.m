function SetupRosView()   
    hold on;
    axis vis3d;
    set(gcf, 'menubar', 'none');
    cameratoolbar('Show');
    cameratoolbar('setcoordsys','z');
    set(gca,'CameraUpVector', [0 0 1] );
    grid on;
    camlight;

    xlabel('x (forward)');
    ylabel('y (left)');
    zlabel('z (up)');
end