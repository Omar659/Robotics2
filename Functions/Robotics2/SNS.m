function [q_SNS, s] = SNS(Q_max, Q_min, J, x_dot)
    % SNS
    %
    % sintax: [q_SNS, s] = SNS(Q_max, Q_min, J, x_dot)
    %
    % input:
    %   Q_max - max velocity value of joints
    %   Q_min - min velocity of joints
    %   J - jacobian
    %   x_dot - desired task velocities
    %
    % output:
    %   q_SNS
    %   s - scaling factor
    
    n = size(J, 2);
    m = size(J, 1);
    q_dot_N = zeros(n, 1);
    W = eye(n);
    s = 1;
    s_star = 0;    
    while 1
        limit_exceed = false;
        % compute the joint velocity with initialized values
        JW = J*W;
        JW_p = pinv(JW);
        q_bar_dot = q_dot_N + JW_p*(x_dot - J*q_dot_N);

        % q_bar_dot step
        vpa(q_bar_dot, 5);

        % check the joint velocity bounds
        ind_max = find(q_bar_dot>Q_max, 1);
        ind_min = find(q_bar_dot<Q_min, 1);
        if not(isempty(ind_min)) || not(isempty(ind_max))
            limit_exceed = true;
            % compute the task scaling factor and the most critical joint
            a = JW_p * x_dot;
            b = q_bar_dot - a;
            [task_scaling_factor, critic_joint] = getTaskScalingFactor(a, b, Q_max, Q_min);
            % if a larger task scaling factor is obtained, save the current solution
            if task_scaling_factor > s_star
                s_star = task_scaling_factor;
                W_star = W;
                q_dot_star_N = q_dot_N;
            end
            % disable the most critical joint by forcing it at its saturated velocity
            j = critic_joint;
            W(j,j) = 0;
            if q_bar_dot(j) > Q_max(j)
                q_dot_N(j) = Q_max(j);
            elseif q_bar_dot(j) < Q_min(j)
                q_dot_N(j) = Q_min(j);
            end
            % check if task can be accomplished with the remaining enabled
            % joints. If NOT, use the parameters that allow the largest
            % task scaling factor and exit
            if rank(JW) < m 
                s = s_star;
                W = W_star;
                q_dot_N = q_dot_star_N;
                q_bar_dot = q_dot_N + JW_p*(s*x_dot - J*q_dot_N);
                limit_exceed = false;
            end
        end
        % repeat until no joint limit is exceeded
        if limit_exceed == false 
            break
        end
    end
    q_SNS = q_bar_dot;
end