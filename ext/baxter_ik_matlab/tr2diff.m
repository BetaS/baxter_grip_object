function d = tr2diff(t1, t2)
    IR = inv(t2)*t1;
    DT = t1(1:3,4)-t2(1:3,4);
    DR = Euler_From_Matrix(IR(1:3,1:3));
    
    for i=1:3
        DR(i) = min(DR(i), abs(DR(i)-pi*2));
    end
    d = [DT; DR'];
end