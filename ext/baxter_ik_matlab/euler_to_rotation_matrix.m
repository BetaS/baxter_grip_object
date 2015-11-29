function result = euler_to_rotation_matrix(R, P, Y)

    RX = [ 1 0 0; 0 cos(R) -sin(R); 0 sin(R) cos(R) ];
    RY = [ cos(P) 0 sin(P); 0 1 0; -sin(P) 0 cos(P)];
    RZ = [ cos(Y) -sin(Y) 0; sin(Y) cos(Y) 0; 0 0 1];

    result = RZ*RY*RX;

end
