clear all;

tp_data = [
        0.069, 0, 0.27;      % s0->s1
        0.102, 0, 0;         % s1->e0
       -0.069, 0, 0.262;   % e0->e1
        0.104, 0, 0;         % e1->w0
       -0.01, 0, 0.271;      % w0->w1
        0.11597, 0, 0;       % w1->w2
];

%baxter postions : The position of each revolute joint in the arm

% Spatial Jacobian
%
q1=[0,0,0,0]'; % for s0 Center Pos
q2=[tp_data(1,1), 0, tp_data(1,3),0]'+q1;
q3=[tp_data(2,1), 0, 0, 0]'+q2;
q4=[tp_data(3,3), 0, tp_data(3,1), 0]'+q3;
q5=[tp_data(4,1), 0, 0, 0]'+q4;
q6=[tp_data(5,3), 0, tp_data(5,1), 0]'+q5;
q7=[tp_data(6,1), 0, 0, 0]'+q6;

w1=[0,0,1]';
w2=[0,1,0]';
w3=[1,0,0]';
w4=w2;
w5=w3;
w6=w2;
w7=w3;

joint_limits = [
    -2.3    0.7; % s0
    -2.0    0.9; % s1
    -2.9    2.9; % e0
    0       2.5; % e1
    -2.9    2.9; % w0
    -1.4    1.9; % w1
    -2.9    2.9; % w2
];

W = [w1 w2 w3 w4 w5 w6 w7];
Q = [q1 q2 q3 q4 q5 q6 q7];

%%
% Jacobian
th1 = [ pi/4 pi/4 0 0 0 0 pi/4 ]';
gst_init = gst(W,Q,th1);

th2 = [ pi/4 0 0 0 0 0 pi/4]';
gst_target = gst(W,Q,th2);

J = Jacobian(W, Q, th2)
    
%dist(4:6) = [0 0 0]
%{

% proof for Forward Kinematics

N = 10000;
th = th1;
th_dot = (th2-th1)/N;
x = gst_init(1:3,4,7);
x(4:6) = Euler_From_Matrix(gst_init(1:3, 1:3, 7))';
for i=1:N
    J = Jacobian(W, Q, th);
    v_dot = J*th_dot;
    
    th = th + th_dot;
    x = x + v_dot;
end
x_d = gst_target(1:3,4,7);
x_d(4:6) = Euler_From_Matrix(gst_target(1:3, 1:3, 7))';
[x x_d]

di = tr2diff(gst_target(:,:,7), gst_init(:,:,7));

eval = [v_dot di]; % should be same

% proof for Inverse Kinematics
J_d = pinv(J);

di = tr2diff(gst_target(:,:,7), gst_init(:,:,7));
q_dot = J_d*di;
q_dot = normalize_angles(q_dot);

ik_gst = gst(W, Q, th1+q_dot);

eval = tr2diff(gst_target(:,:,7), ik_gst(:,:,7)); % should be zero

%{
th_dot = J_d*v_dot
th2 = th1+J_d*v_dot;

en = gst(W, Q, th2);
x2 = [en(1:3,4,7); Euler_From_Matrix(en(1:3,1:3,7))']
%}

%}
%{/
N = 1000;
dt = 1/100;%100/N;%1/100;
th(:,1) = th1;
g = [];
g(:,:,:,1) = gst(W, Q, th(:,1));
dist = [];

for i=1:N
    dist(:,i) = tr2diff(gst_target(:,:,7), g(:,:,7,i));
    %dist(1:3,i) = gst_target(1:3,4,7)-g(1:3,4,7,i);
    dist(4:6,i) = [0 0 0]';
    n_dist(:, i) = norm(dist(:, i));
    
    J = Jacobian(W, Q, th(:,i));
    J_d = pinv(J);
    
    if n_dist < 1e-2
        break
    end

    di = dist(:,i)*dt;

    theta_dot = J_d*di;
    th(:,i+1) = th(:,i)+(theta_dot);
    
    %{/
    for j=1:7
        angle = th(j,i+1);
        th(j,i+1) = atan2(sin(angle), cos(angle));
    end
    %}/
    %joint limits
    %{
    for j = 1:7
        if th(j,i+1) < joint_limits(j, 1)
            th(j,i+1) = 0;%joint_limits(j, 1);
        elseif th(j,i+1) > joint_limits(j, 2)
            th(j,i+1) = 0;%joint_limits(j, 2);
        end
    end
    %}
    
    g(:,:,:,i+1) = gst(W, Q, th(:,i+1));
end

%}/
%%
%{/
% Sim mode

figure(1)
% Print distance rate
plot(1:length(dist), dist(:, 1:length(dist)));
hold on
plot(1:length(n_dist), n_dist(:, 1:length(n_dist)), 'linewidth', 2, 'color', 'r');
hold off
xlim([1 length(dist)]);
drawnow
    
for k = 1:length(th)
    figure(2)
    
    pba = get(gca, 'PlotBoxAspectRatio');
    dar = get(gca, 'DataAspectRatio');
    cva = get(gca, 'CameraViewAngle');
    cuv = get(gca, 'CameraUpVector');
    ct = get(gca, 'CameraTarget');
    cp = get(gca, 'CameraPosition');
    
    plot3([0 0 0], [0 0 0], [0 0 0])
    
    set(gca, 'PlotBoxAspectRatio',pba);
    set(gca, 'DataAspectRatio',dar);
    set(gca, 'CameraViewAngle',cva);
    set(gca, 'CameraUpVector',cuv);
    set(gca, 'CameraTarget',ct);
    set(gca, 'CameraPosition',cp);
    
    grid on
    xlim([-1.5 1.5]);
    ylim([-1.5 1.5]);
    set(gca,'xtick',-1.5:0.2:1.5)
    set(gca,'ytick',-1.5:0.2:1.5)   
    zlim([-1 2]);

    % Print planned joint angles
    draw_gst(g(:,:,:,k), 3, 'r');
    %draw_gst(gst_target, 2, 'b');
    
    % Draw Velocity
    v = dist(:, k);
    pos = g(1:3,4,7,1:k);
    %hold on
    %plot3([pos(1) pos(1)+v(1)], [pos(2) pos(2)+v(2)], [pos(3) pos(3)+v(3)], 'linewidth', 2, 'color', 'g')
    %hold off
    hold on
    plot3(pos(1,:), pos(2, :), pos(3, :), 'linewidth', 2, 'color', 'c')
    hold off
    
    drawnow
end
%}/