function [theta,x,dist]=inv_kin_position2(w, q, x_d, q1)

x = [];
theta = [];
dist = [];

theta(:,1)=q1; % inital configuration, q_pos(k) is 7x1

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
stol = 1e-4;
dt = 1/N;
k = 1;
dist(1) = 1;

trs = gst(w, q, q1);
tr = trs(:,:,7);
    
while dist(k) > stol
    trs = gst(w, q, theta(:,k));
    tr = trs(:,:,7);
    
    %di = x_d_tr(1:3,4)-tr(1:3,4);%tr2diff(x_d_tr, tr);
    di(1:3) = [1 1 -0.1];
    di(4:6) = [0 0 0];      % Orientation
    x(:,:,:,k) = trs(:,:,:);
    
    %{
    % Update Twist Point
    for i=1:7
        q(1:3,i) = trs(1:3,4,i);
    end
    %}
    
    J = Jacobian(w, q, theta(:,k));
    J_d = pinv(J);
    
    %v(:,k) = 2*di/((N+1-k)*dt);
    v(:,k) = di*dt;%di*dt;
    
    %solving for the joint rates at planned velocity
    q_dot = J_d*(v(:,k));
    %q_dot = J_d*(v(:,k)/dt);
    %numerical integration
    theta(:,k+1) = theta(:,k)+q_dot;
    %{/
    for i=1:7
        angle = theta(i,k+1);
        theta(i,k+1) = atan2(sin(angle), cos(angle));
    end
    %}/
    %joint limits
    %{/
    for j = 1:7
        if theta(j,k+1) < joint_limits(j, 1)
            theta(j,k+1) = joint_limits(j, 1);
        elseif theta(j,k+1) > joint_limits(j, 2)
            theta(j,k+1) = joint_limits(j, 2);
        end
    end
    %}/
    
    dist(k+1) = norm(di);
    
    k = k+1;
    
    if k > N
        'Solution wouldn''t converge'
        %msgbox('Solution wouldn''t converge')
        break
    end
    
end
end
