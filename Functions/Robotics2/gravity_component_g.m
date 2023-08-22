function g_fun = gravity_component_g(q, DH_matrices, i_r_ci, m, g, moving_frame)
    % gravity_component_g - compute the gravity component of the
    %                       Euler-Lagrange equation
    %
    % sintax: g_fun = gravity_component_g(q, DH_matrices, i_r_ci, m, g, moving_frame)
    %
    % input:
    %   q - joint positions
    %   DH_matrices - DH matrix of each link
    %   i_r_ci - COM position of each link
    %   m - vector of link masses
    %   g - vector of gravity
    %   moving_frame - if true I need to compute the position of the COM
    %                  otherwise is already computed and ready to use in i_r_ci
    %
    % output:
    %   g_fun - gravity term of the Euler-Lagrange equation

    zero_DH_i = eye(4);
    U_stack = [] * q(1);
    for i = 1:length(q)
        if moving_frame
            % DH matrices
            j_DH_i =  DH_matrices{i};
            % 0^A_1 * 1^A_2 * ... * {i-1}^A_i = 0^A_i
            zero_DH_i = zero_DH_i * j_DH_i;
            % homogeneous coordinate of COM i wrt to reference frame 0
            zero_r_ci = zero_DH_i * [i_r_ci(:, i); 1];
            % potential energy of link i
            Ui = simplify(-m(i) * g' * zero_r_ci(1:3));
        else
            Ui = simplify(-m(i) * g' * i_r_ci(:, i));
        end
        U_stack = [U_stack Ui];
    end     
    % global potential energy
    U = simplify(sum(U_stack));
    % gravity term
    g_fun = simplify(jacobian(U, q)');
end