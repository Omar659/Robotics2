function c = christoffel_component_c(q, q_dot, M)
    % christoffel_component_c - compute the christoffel symbols
    %
    % sintax: c = christoffel_component_c(q, q_dot, M)
    %
    % input:
    %   q - joint positions
    %   q_dot - joint velocities
    %   M - inertia matrix component of the Euler-Lagrange equation
    %
    % output:
    %   c - christoffel symbols term of the Euler-Lagrange equation

    % Compute the christoffel symbols term of the Euler-Lagrange equation
    c = []*q(1);
    for i = 1:length(q_dot)
        M_i = M(:, i);
        Ci = simplify(1/2*(jacobian(M_i, q) + jacobian(M_i, q)' - diff(M, q(i))));
        c_i = simplify(q_dot' * Ci * q_dot);
        c = [c; c_i]; 
    end
end