function draw_end_point( gst, size, color )

    pos = gst(1:3,4,:);
    Rm = gst(1:3,1:3,:);
    
    axisx = Rm*[0.2    0     0]';
    axisy = Rm*[0       0.2  0]';
    axisz = Rm*[0       0     0.2]';
        
    hold on
    % X-Axis
    plot3([pos(1) pos(1)+axisx(1)], [pos(2) pos(2)+axisx(2)], [pos(3) pos(3)+axisx(3)], 'linewidth', 1, 'color', 'm');
    hold off

    hold on
    % Y-Axis
    plot3([pos(1) pos(1)+axisy(1)], [pos(2) pos(2)+axisy(2)], [pos(3) pos(3)+axisy(3)], 'linewidth', 1, 'color', 'y');
    hold off

    hold on
    % Z-Axis
    plot3([pos(1) pos(1)+axisz(1)], [pos(2) pos(2)+axisz(2)], [pos(3) pos(3)+axisz(3)], 'linewidth', 1, 'color', 'c');
    hold off
    
    hold on
    [x,y,z] = sphere();
    surf(x*0.02+pos(1),y*0.02+pos(2),z*0.02+pos(3), 'FaceColor', 'k');
    hold off
end

