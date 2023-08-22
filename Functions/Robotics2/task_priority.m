function q_dot_j = task_priority(J_all, r_dot_all)
    % task_priority - compute the joints velocity prioritizing the tasks
    %
    % sintax: q_dot_j = task_priority(J_all, r_dot_all)
    %
    % input:
    %   J_all - array of jacobian of the tasks ordered by priority
    %   r_dot_all - array of task velocities ordered by priority
    %
    % output:
    %   q_dot_j - the joints velocity at the end of the recursion

    % j = k-1
    % inizialization
    q_dot_j = zeros(size(J_all{1}, 2), 1);
    P_j = eye(size(J_all{1}, 2));
    for k = 1 : length(J_all)
        % take the first k J and r_dot and stack vertically
        J_k = vertcat(J_all{1:k});
        r_dot_k = vertcat(r_dot_all{1:k});

        % recursive step for P (projector in the null space)
        P_k = P_j - pinv(J_k*P_j)*J_k*P_j;
        % recursive step for q_dot
        q_dot_k = q_dot_j + pinv(J_k*P_j)*(r_dot_k - J_k*q_dot_j);

        % j = k
        % k = k + 1
        % vpa the results
        P_j = vpa(P_k, 5);
        q_dot_j = vpa(q_dot_k, 5);
    end

end