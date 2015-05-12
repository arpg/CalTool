function SetupVisionView()   
    hold on;
    view(3);
    axis vis3d;
    set(gcf, 'menubar', 'none');
    cameratoolbar('SetMode','orbit')
    cameratoolbar('Show');
    cameratoolbar('setcoordsys','y');
    set(gca,'CameraUpVector', [0 -1 0] );
    grid on;

    xlabel('x (right)');
    ylabel('y (down)');
    zlabel('z (forward)');
end
