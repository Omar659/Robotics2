function [v_ci, omega] = moving_frame_algorithm(q, q_dot, DH_matrices, sigma, i_r_ci)
    % moving_frame_algorithm - compute the velocities (angular and linear)
    %                          of the COM of each link
    %
    % sintax: [v_ci, omega] = moving_frame_algorithm(q, q_dot, DH_matrices, sigma, i_r_ci)
    %
    % input:
    %   q - joint positions
    %   q_dot - joint velocities
    %   DH_matrices - DH matrix of each link
    %   sigma - vector of 0 (if revolute joint) and 1 (if prismatic joint)
    %   i_r_ci - COM position of each link
    %
    % output:
    %   v_ci - Vector of linear velocities of centers of mass
    %   omega - Vector of angular velocities of centers of mass

    % base rotation. Fixed base, initialized at zero
    j_omega_j = [0 0 0]';
    % base velocity. Fixed base, initialized at zero
    j_v_j = [0 0 0]';
    omega = [];
    v_ci = [];
    for i=1:length(q)
        % DH_matrices -> rotation and translation components
        j_DH_i = DH_matrices{i};
        j_r_ji = j_DH_i(1:3, 4);
        j_R_i = j_DH_i(1:3, 1:3);

        % compute angular speed of the link i COM
        j_omega_i = simplify(j_omega_j + (1 - sigma(i))*q_dot(i)*[0 0 1]');
        i_omega_i = simplify(j_R_i' * j_omega_i);
        omega = [omega i_omega_i];

        % compute linear speed of the link i
        i_v_i = simplify(j_v_j + sigma(i)*q_dot(i)*[0 0 1]' + cross(j_omega_i, j_r_ji));
        i_v_i = simplify(j_R_i' * i_v_i);

        % compute linear speed of the link i COM
        i_v_ci = simplify(i_v_i + cross(i_omega_i, i_r_ci(:, i)));
        v_ci = [v_ci i_v_ci];

        % update for the next iteration
        j_omega_j = i_omega_i;
        j_v_j   = i_v_i;
    end
end