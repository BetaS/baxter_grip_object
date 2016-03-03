function [Jst] = Jacobian_Origin(w, q, theta)
q(end,:) = [];
[~,n] = size(w);
% xi
for i = 1:n
    xi{i} = [-cross(w(:,i),q(:,i)); w(:,i)];
end
% xih
for i = 1:n
    xih{i} = [hat(w(:,i)) -cross(w(:,i),q(:,i))];
end;
% exi
for i = 1:n
    ew{i} = eye(3) + hat(w(:,i))*sin(theta(i)) + hat(w(:,i))^2*(1-cos(theta(i)));
    v = -cross(w(:,i),q(:,i));
	exi{i} = [ew{i} (eye(3)-ew{i})*cross(w(:,i),v)+ w(:,i)*w(:,i)'*v*theta(i); zeros(1,3) 1];
end;
% Jacobian.
Jst(:,1) = xi{1};
Exi = eye(4);
for i = 1:(n-1)
    Exi = Exi*exi{i};
    R = Exi(1:3,1:3);
    P = Exi(1:3,4);
    Adg{i} = [R hat(P)*R; zeros(3,3) R];
    Jst(:,(i+1)) = Adg{i}*xi{i+1};
end;