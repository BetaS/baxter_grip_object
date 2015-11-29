function R = gst(w, q, theta)
e = [];
q(4,:) = [];
for i=1:7
    e(:,:,i) = expon(w(:,i), q(:,i), theta(i));
end

for i=1:7
    % gst(0)
    Rst0 = eye(3);%[0 0 1; 0 1 0; -1 0 0];
    %Rst0 = [0 0 1; 0 1 0; 1 0 0];
    
    g0=[];
    g0(1:3, 1:3)=Rst0;
    g0(1:3, 4) = q(:,i);
    g0(4,:)=[0 0 0 1];

    % products of e_xi_theta : (4x4)-matrix
    Rm = eye(4);

    for j = 1:i          
        Rm = Rm * e(:,:,j);
    end
    R(:,:,i) = Rm * g0;
end

end