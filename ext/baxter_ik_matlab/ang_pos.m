function p = ang_pos(theta)
% This function return position of end effector based on the angle of
% joints
l0 = 0.27035;
l1 = 0.36435;
l2 = 0.37429;
l3 = 0.229525;
g0 = [eye(3) [0; l1+l2+l3; l0]; zeros(1,3) 1];
w1 = [0; 0; 1];
w2 = [-1; 0; 0];
w3 = [0; 1; 0];
w4 = [-1; 0; 0];
w5 = [0; 1; 0];
w6 = [-1; 0; 0];
w7 = [0; 1; 0];
q1 = [0; 0; l0; 1];
q2 = [0; l1; l0; 1];
q3 = [0; l1+l2; l0; 1];
e1 = expon(w1, q1, theta(1));
e2 = expon(w2, q1, theta(2));
e3 = expon(w3, q1, theta(3));
e4 = expon(w4, q2, theta(4));
e5 = expon(w5, q2, theta(5));
e6 = expon(w6, q3, theta(6));
e7 = expon(w7, q3, theta(7));
gd = e1*e2*e3*e4*e5*e6*e7*g0;
p = gd(1:3,4);

end