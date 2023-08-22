function [M_p, c_p, g_fun_p, u_p] = coordinate_transformation(q, q_dot, f_inv, p_dot, p_dot_dot, J, M, c, g_fun)
    % coordinate_transformation - change the coordinate in a new set of 
    %                             generalized coordinates p. 
    %                             P.S The robot is not redundant and it is 
    %                             not in a singular case
    %
    % sintax: [M_p, c_p, g_fun_p, u_p] = coordinate_transformation(q, q_dot, f_inv, p_dot, p_dot_dot, J, M, c, g_fun)
    %
    % input:
    %   q - joint positions (symbols)
    %   q_dot - joint velocities (symbols)
    %   f_inv - inverse kinematic (formula)
    %   p_dot - task velocities (symbols)
    %   p_dot_dot - task accelerations (symbols)
    %   J - jacobian of f
    %   M - inertia matrix component of the Euler-Lagrange equation
    %   c - christoffel symbols term of the Euler-Lagrange equation
    %   g_fun - gravity term of the Euler-Lagrange equation
    %
    % output:
    %   M_p - inertia matrix component of the Euler-Lagrange equation in 
    %         the new coordinates
    %   c_p - christoffel symbols term of the Euler-Lagrange equation in 
    %         the new coordinates
    %   g_fun_p - gravity term of the Euler-Lagrange equation in the new 
    %             coordinates
    %   u_p - torque from the Euler-Lagrange equation in the new 
    %         coordinates

    % utility
    J_inv = inv(J);
    J_dot = zeros(size(J, 1), size(J, 2));
    for i = 1:length(q)
        J_dot = J_dot + diff(J_dot, q(i))*q_dot(i);
    end
    f_dot_inv = J_inv * p_dot;

    % new M
    M_p = simplify(J_inv' * M * J_inv);
%     M_p = simplify(inv(J * inv(M) * J')); % caso ridondante ho dubbi sul funzionamento
    M_p = subs(M_p, q, f_inv);
    
    % new c
    c_p = simplify(J_inv'*c - M_p*J_dot*J_inv*p_dot);
%     c_p = simplify(M_p*(J*inv(M)*c - J_dot*q_dot)); % caso ridondante ho dubbi sul funzionamento
    c_p = subs(c_p, q, f_inv);
    c_p = subs(c_p, q_dot, f_dot_inv);

    % new g_fun
    g_fun_p = simplify(J_inv' * g_fun);
%     g_fun_p = simplify(M_p*J*inv(M)*g_fun); % caso ridondante ho dubbi sul funzionamento
    g_fun_p = subs(g_fun_p, q, f_inv);
    
    % new u
    u_p = simplify(M_p*p_dot_dot + c_p + g_fun_p);
end