function pose = dir_kin(w, q, theta)
e = [];
for i=1:7
    e{i} = expon(w(:,i), q(:,i), theta(i));
end

q(4,:) = [];

% gst(0)
Rst0 = eye(3);%[0 0 1; 0 1 0; -1 0 0];
g0=[];
g0(1:3, 1:3)=Rst0;
g0(1:3, 4) = q(:,i);
g0(4,:)=[0 0 0 1];

% products of e_xi_theta : (4x4)-matrix
Rm = eye(4);

for j = 1:7        
    Rm = Rm * e{j};
end
R = Rm * g0;

euler = Euler_From_Matrix(R(1:3,1:3));
pose(1) = R(1,4);
pose(2) = R(2,4);
pose(3) = R(3,4);
pose(4) = euler(1);
pose(5) = euler(2);
pose(6) = euler(3);
end