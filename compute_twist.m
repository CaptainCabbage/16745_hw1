function [twist] = compute_twist(omega,p)
v = -cross(omega,p);
twist = [v;omega];

end

