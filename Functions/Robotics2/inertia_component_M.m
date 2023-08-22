function M = inertia_component_M(q, q_dot, v_ci, omega, I, m)
    % inertia_component_M - compute the inertia matrix component of the
    %                       Euler-Lagrange equation
    %
    % sintax: M = inertia_component_M(q, q_dot, v_ci, omega, I, m)
    %
    % input:
    %   q - joint positions
    %   q_dot - joint velocities
    %   DH_matrices - DH matrix of each link
    %   v_ci - Vector of linear velocities of centers of mass
    %   omega - Vector of angular velocities of centers of mass
    %   I - link inertia matrices
    %   m - vector of link masses
    %
    % output:
    %   M - inertia matrix component of the Euler-Lagrange equation

    T_stack = []*q(1);
    for i=1:length(q)
        % linear speed kinetic energy
        ke_v = (1/2) * m(i) * (v_ci(:, i)' * v_ci(:, i));
        % angular speed kinetic energy
        ke_w = (1/2) * (omega(:, i)' * I(:,:,i) * omega(:, i));
        % kinetic energy of link i
        Ti = simplify(ke_v + ke_w);
        T_stack = [T_stack Ti];
    end         

    % total kinetic energy
    T = simplify(sum(T_stack));
        
    % inertia matrix M
    M = simplify(hessian(T, q_dot));
end