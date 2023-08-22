function J_p = interaction_matrix(u, v, Z, lambda)
    % interaction_matrix - interaction matrix
    %
    % sintax: J_p = interaction_matrix(u, v, Z, lambda)
    %
    % input:
    %   u - x coordinate of the image plane
    %   v - y coordinate of the image plane
    %   lambda - focal length
    %
    % output:
    %   J_p - interaction matrix
    
    J_p = [-(lambda/Z)  0          u/Z u*v/lambda            -(lambda + (u^2)/lambda)  v;
            0          -(lambda/Z) v/Z lambda + (v^2)/lambda -u*v/lambda              -u];
end