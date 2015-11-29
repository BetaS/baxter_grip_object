function Euler = Euler_From_Matrix(m)
    Euler(2) = atan2(-m(3,1), sqrt(m(1,1)^2+m(2,1)^2));
    Euler(1) = atan2(m(3,2)/cos(Euler(2)), m(3,3)/cos(Euler(2)));
    Euler(3) = atan2(m(2,1)/cos(Euler(2)), m(1,1)/cos(Euler(2)));
end