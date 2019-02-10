function [J] = compute_Jacobians(thetas,twists)

N = numel(thetas);
J = zeros(6,N);

for i = 1:N 
    if i == 1
        xi_ = twists(:,i);
    else 
        twist = twists(:,i);
        theta = thetas(i);
        v = twist(1:3);
        omega = twist(4:6);
        omega_hat = [0,-omega(3),omega(2);omega(3),0,-omega(1);-omega(2),omega(1),0];
        twist_hat = [omega_hat,v;0 0 0 0];
        xi_hat = twist_hat;

        for j = 1:i-1
            e = exp_twist(twists(:,j),thetas(j));
            xi_hat = e*xi_hat*inv(e);
        end

        xi_ = [xi_hat(3,2);xi_hat(1,3);xi_hat(2,1);xi_hat(1:3,4)];
    end
    
    J(:,i) = xi_;

end

