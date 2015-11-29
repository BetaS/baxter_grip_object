% J is Body Jacobian, which is a (6x7)-matrix
% Body Jacobian is for each joint or link where the reference frame is
% placed

function [J] = Body_Jacobian(w, q, theta)


    % Rotation Matrix 
    g = gst(w, q, theta);
    
    %q = [-q(:,7)-q(:,1), -q(:,7)-q(:,2), -q(:,7)-q(:,3), -q(:,7)-q(:,4), -q(:,7)-q(:,5), -q(:,7)-q(:,6), -q(:,7)-q(:,7)];

    % xi
    for i = 1:7
        xi{i} = [-cross(w(:,i),q(1:3,i)); w(:,i)];
    end
    
    % For making Jacobian, we need xi_cross : (6x7)-matrix
    xi_cross=[];

    for i=1:7
        % products of e_xi_theta : (4x4)-matrix
        Exi = g(:,:,i);
        R = Exi(1:3,1:3);
        P = Exi(1:3,4);
        Adgi = [R' -1*R'*hat(P); zeros(3,3) R'];
    
        xi_cross(:,i) = Adgi * xi{i}; % not sure... 
    end

    % J : (6x7)-matrix
    J=[];
    for k=1:7
        J(:,k) = xi_cross(:,k);
    end

end