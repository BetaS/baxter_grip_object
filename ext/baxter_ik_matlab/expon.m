function [exi] = expon(w, q, theta)
% This funtion calculates the exponential for xi.
q = q(1:3);
wh = hat(w);
v = -cross(w, q);
ew = eye(3) + wh*sin(theta) + (1 - cos(theta))*wh^2;
p = (eye(3) - ew)*cross(w, v) + w*w'*v*theta;
exi = [ew p; 0 0 0 1];
end