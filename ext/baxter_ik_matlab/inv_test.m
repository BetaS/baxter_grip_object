clear all;

tp_data = [
        0.069, 0, 0.27;      % s0->s1
        0.102, 0, 0;         % s1->e0
        0.069, 0, 0.262;   % e0->e1
        0.104, 0, 0;         % e1->w0
       -0.01, 0, 0.271;      % w0->w1
        0.11597, 0, 0;       % w1->w2
];

%baxter postions : The position of each revolute joint in the arm

q1=[0,0,0,0]'; % for s0 Center Pos
q2=[tp_data(1,1), 0, tp_data(1,3),0]'+q1;
q3=[tp_data(2,1), 0, 0, 0]'+q2;
q4=[tp_data(3,3), 0, -tp_data(3,1), 0]'+q3;
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

W = [w1 w2 w3 w4 w5 w6 w7];
Q = [q1 q2 q3 q4 q5 q6 q7];


%%
% Jacobian
th1 = [ 0 0 0 0 0 0 0 ]';
x1 = dir_kin(W,Q,th1)';

gst_init = gst(W,Q,th1);

th2 = [-pi/6, 0, 0, 0, 0, 0, 0]%[-1.7 -1.0159, 1.3, 1.4, 1.4, 0.8, 1.1415]';
x2 = dir_kin(W,Q,th2)';

gst_target = gst(W,Q,th2);

gst_init(1:3,4,7)
gst_target(1:3,4,7)

[q,x,dist] = inv_kin_position(W, Q, x2, th1);

%%
%{
% Result mode

figure(1)
% Print distance rate
plot(1:length(dist), dist(:));

figure(2)
% Print planned joint angles

grid on
xlim([-0.5 1.5]);
ylim([-0.5 1.0]);
set(gca,'xtick',-0.5:0.2:1.5)
set(gca,'ytick',-1.0:0.2:1.0)   
zlim([-1 1]);

draw_gst(x(:,:,:,length(x)), 3, 'r');
draw_gst(gst_init, 2, 'b');
draw_end_point(gst_target(:,:,7), 2);
    
hold on
endpoint = x(:,:,7,length(x));
endpoint = endpoint(1:3,4);
% Print end-effector trajectory
%plot3(endpoint(1,:),endpoint(2,:),endpoint(3,:),'linewidth', 1)
hold off


drawnow
hold off
%}

%%
%{/
% Sim mode
for k = 1:length(dist)-1
    figure(1)
    % Print distance rate
    plot(1:k, dist(1:k));
    xlim([1 1000]);
    drawnow
    
    figure(2)
    
    clf(2, 'reset');
    
    view(-25, 25);
    
    grid on
    xlim([-1.2 1.2]);
    ylim([-1.2 1.2]);
    set(gca,'xtick',-1.2:0.2:1.2)
    set(gca,'ytick',-1.2:0.2:1.2)   
    zlim([-1 1]);

    % Print planned joint angles
    gst = x(:,:,:,k);
    draw_gst(gst, 3, 'r');
    %draw_gst(gst_init, 2, 'b');
    draw_end_point(gst_target(:,:,7), 2);
    
    endpoint = x(:,:,7,k);
    endpoint = endpoint(1:3,4);
    % Print end-effector trajectory
    %plot3(endpoint(1,:),endpoint(2,:),endpoint(3,:),'linewidth', 2)

    % pause(1)
    drawnow
end
%}/