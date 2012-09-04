function [ R ] = AngleAxisToRotMatrix( u, theta )
% exponential map
% a rotation around axis u by angle θ.

R = expm(cross(u/norm(u))*theta);

end

