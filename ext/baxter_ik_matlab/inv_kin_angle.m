function [q_pos,x,dist]=inv_kin_position2(w, q, x_d, q1)

x = [];
q_pos = [];
dist = [];

q_pos(:,1)=q1; % inital configuration, q_pos(k) is 7x1

joint_limits = [
    -2.3    0.7; % s0
    -2.0    0.9; % s1
    -2.9    2.9; % e0
    0       2.5; % e1
    -2.9    2.9; % w0
    -1.4    1.9; % w1
    -2.9    2.9; % w2
];

x_d_tr = [];
x_d_tr(1:3,1:3) = euler_to_rotation_matrix(x_d(4), x_d(5), x_d(6));
x_d_tr(1:3,4) = x_d(1:3);
x_d_tr(4,1:4) = [0 0 0 1];

N = 1000;
stol = 1e-2;
dt = 100/N;
k = 1;
dist(1) = 1;

while dist(k) > stol
    trs = gst(w, q, q_pos(:,k));
    tr = trs(:,:,7);
    di = tr2diff(x_d_tr, tr);
    x(:,:,k) = trs(1:3,4,:);
    
    J = Jacobian(w, q, q_pos(:,k));
    J_d = pinv(J);
    
    %v(:,k) = 2*di/((N+1-k)*dt);
    v(:,k) = di*dt;
    
    %solving for the joint rates at planned velocity
    q_dot = J_d*v(:,k);
    %numerical integration
    q_pos(:,k+1) = q_pos(:,k)+q_dot;
    
    for i=1:7
        angle = q_pos(i,k+1);
        q_pos(i,k+1) = atan2(sin(angle), cos(angle));
    end
    
    %joint limits
    %{
    for j = 1:7
        if q(j,k+1) < joint_limits(j, 1)
            q(j,k+1) = joint_limits(j, 1);
        elseif q(j,k+1) > joint_limits(j, 2)
            q(j,k+1) = joint_limits(j, 2);
        end
    end
    %}
    
    dist(k+1) = norm(di);
    
    k = k+1;
    
    if k > N
        'Solution wouldn''t converge'
        %msgbox('Solution wouldn''t converge')
        break
    end
    
end
end
