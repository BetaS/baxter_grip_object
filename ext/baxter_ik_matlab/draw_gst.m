function draw_gst( gsts , size, color )
    pos = gsts(1:3,4,:);
    
    for i=1:length(gsts)
        Rm = gsts(1:3,1:3,i);
        axisx = Rm*[0.1    0     0]';
        axisy = Rm*[0       0.1  0]';
        axisz = Rm*[0       0     0.1]';
        
        hold on
        % X-Axis
        plot3([pos(1,i) pos(1,i)+axisx(1)], [pos(2,i) pos(2,i)+axisx(2)], [pos(3,i) pos(3,i)+axisx(3)], 'linewidth', 1, 'color', 'r');
        hold off
    
        hold on
        % Y-Axis
        plot3([pos(1,i) pos(1,i)+axisy(1)], [pos(2,i) pos(2,i)+axisy(2)], [pos(3,i) pos(3,i)+axisy(3)], 'linewidth', 1, 'color', 'g');
        hold off
        
        hold on
        % Z-Axis
        plot3([pos(1,i) pos(1,i)+axisz(1)], [pos(2,i) pos(2,i)+axisz(2)], [pos(3,i) pos(3,i)+axisz(3)], 'linewidth', 1, 'color', 'b');
        hold off
    end
    
    hold on
    % Print  joint angles - for test
    plot3(pos(1,:), pos(2,:), pos(3,:), 'linewidth', size, 'color', color);
    
    hold off

end

