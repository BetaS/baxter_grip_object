function [Jst] = Jacobian(w, q, theta)
Gst = gst(w, q, theta);

q(end,:) = [];
[~,n] = size(w);

q_diff = [Gst(1:3,4,7)-Gst(1:3,4,1), Gst(1:3,4,7)-Gst(1:3,4,2), Gst(1:3,4,7)-Gst(1:3,4,3), Gst(1:3,4,7)-Gst(1:3,4,4), Gst(1:3,4,7)-Gst(1:3,4,5), Gst(1:3,4,7)-Gst(1:3,4,6), Gst(1:3,4,7)-Gst(1:3,4,7)];


% xi
for i = 1:n
    xi{i} = [cross(w(:,i),q_diff(1:3,i)); w(:,i)];%[-cross(w(:,i),q(:,i)); w(:,i)];
end

% xih
%{
for i = 1:n
    xih{i} = [hat(w(:,i)) -cross(w(:,i),q(:,i))];
end;
%}
% exi
for i = 1:n
	exi{i} = expon(w(:,i), q(:,i), theta(i));
end;

% Jacobian.
Jst = [];
for i = 1:n
    G = Gst(:,:,i);
    R = G(1:3,1:3);
    P = q_diff(:,i);
    Adg = [R                hat(P)*R; 
                zeros(3,3)  R];
    Jst(:,i) = Adg*xi{i};
end;