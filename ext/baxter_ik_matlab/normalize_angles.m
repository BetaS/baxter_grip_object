function angles = normalize_angles(angles)
    for i=1:length(angles)
        angle = angles(i);
        angles(i) = atan2(sin(angle), cos(angle));
    end
end