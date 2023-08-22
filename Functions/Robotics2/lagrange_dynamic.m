function [M, c, g_fun, u, Y] = lagrange_dynamic(q, q_dot, q_dot_dot, m, g, I,  i_r_ci, sigma, alpha, theta, a, d, v_ci, omega)
    % lagrange_dynamic - compute the torque using the Euler-Lagrange
    % equation
    %
    % sintax: [M, C, g_fun, u, Y] = lagrange_dynamic(q, q_dot, q_dot_dot, m, g, I,  i_r_ci, sigma, alpha, theta, a, d, v_ci, omega)
    %
    % input:
    %   q - joint positions (symbols)
    %   q_dot - joint velocities (symbols)
    %   q_dot_dot - joint accelerations (symbols)
    %   m - vector of link masses
    %   g - vector of gravity
    %   I - link inertia matrices
    %   i_r_ci - COM position of each link
    %   sigma - vector of 0 (if revolute joint) and 1 (if prismatic joint)
    %           for the moving frame algorithm. If global coordinates are 
    %           used then this value is irrelevant
    %   alpha - vector of "alpha" component of the DH table. If global 
    %           coordinates are used then this value is irrelevant
    %   theta - vector of "theta" component of the DH table. If global 
    %           coordinates are used then this value is irrelevant
    %   a - vector of "a" component of the DH table. If global coordinates 
    %       are used then this value is irrelevant
    %   d - vector of "d" component of the DH table. If global coordinates 
    %       are used then this value is irrelevant
    %   v_ci - Vector of linear velocities of centers of mass. This 
    %          parameter has a value only if the moving frame algorithm is 
    %          not used
    %   omega - Vector of angular velocities of centers of mass. This 
    %           parameter has a value only if the moving frame algorithm is 
    %           not used
    %
    % output:
    %   M - inertia matrix component of the Euler-Lagrange equation
    %   c - christoffel symbols term of the Euler-Lagrange equation
    %   g_fun - gravity term of the Euler-Lagrange equation
    %   u - torque from the Euler-Lagrange equation
    %   Y - regressor matrix for the coefficients "a". If it is not
    %       calculated return []

    % Compute DH matricese for the moving frame algorithm
    DH_matrices = {};
    moving_frame = false;
    if nargin == 12
        moving_frame = true;
        for i = 1:length(q)
            j_DH_i = dh_matrix(a(i), alpha(i), d(i), theta(i));
            DH_matrices = [DH_matrices, {j_DH_i}];
        end
        [v_ci, omega] = moving_frame_algorithm(q, q_dot, DH_matrices, sigma, i_r_ci);
    end
    % Compute the gravity term of the Euler-Lagrange equation
    g_fun = gravity_component_g(q, DH_matrices, i_r_ci, m, g, moving_frame);
    % Compute the inertia matrix term of the Euler-Lagrange equation
    M = inertia_component_M(q, q_dot, v_ci, omega, I, m);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % I calculate the values "a_i" by seeing them by eye from M and g_fun 
    % and then I rewrite M and g_fun in such a way that the results are 
    % simplified and as a function of "a_i". Also so I can create the 
    % vector "a" with which I can calculate the regressor 
    % Y(q, q_dot, q_dot_dot) such that Y*a = u

%     syms a1 a2 a3 a4 a5 q1 q2 q3 real
%     a = [a1 a2 a3 a4 a5]';
%     M = [2*a1*cos(q(2)) + a3 a1*cos(q(2)) + a2;
%          a1*cos(q(2)) + a2   a2];
%     
%     g_fun = [a4*cos(q(1)+q(2)) + a5*cos(q(1));
%              a4*cos(q(1)+q(2))];

%     viscous friction
%     Fv = [a4; a5].*q_dot;    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Compute the christoffel symbols term of the Euler-Lagrange equation
    c = christoffel_component_c(q, q_dot, M);

    % Compute torque from the Euler-Lagrange equation
    u = simplify(M*q_dot_dot + c + g_fun);

    % Regressor Y
    Y = [];
%     Y = simplify(jacobian(u + Fv, a));
%     Y = simplify(jacobian(u, a));

    % For adaptive controller (dipende dall'esercizio e c è sbagliato va
    % rifatto)

%     S = []*q(1);
%     for i = 1:length(q_dot)
%         M_i = M(:, i);
%         Ci = simplify(1/2*(jacobian(M_i, q) + jacobian(M_i, q)' - diff(M, q(i))));
%         S_i = simplify(q_dot' * Ci);
%         S = [S; S_i]; 
%     end
%     syms q_dot_1_r q_dot_2_r q_dot_3_r q_dot_dot_1_r q_dot_dot_2_r q_dot_dot_3_r real 
%     q_dot_dot_r = [q_dot_dot_1_r q_dot_dot_2_r q_dot_dot_3_r]';
%     q_dot_r = [q_dot_1_r q_dot_2_r q_dot_3_r]';
%     u = simplify(M*q_dot_dot_r + S*q_dot_r + g_fun);
%     Fv = [a4; a5].*q_dot_r;
%     Y = simplify(jacobian(u, a));
end



