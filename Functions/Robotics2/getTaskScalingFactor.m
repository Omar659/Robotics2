function [task_scaling_factor,critic_joint] = getTaskScalingFactor(a,b, Q_max, Q_min)
    % getTaskScalingFactor
    %
    % sintax: [task_scaling_factor,critic_joint] = getTaskScalingFactor(a,b, Q_max, Q_min)
    %
    % input:
    %   a = JW_p * x_dot;
    %   b = q_bar_dot - a; 
    %   Q_max - max value of joints
    %   Q_min - min value of joints
    %
    % output:
    %   task_scaling_factor - scaling factor
    %   critic_joint - most violate constraint joint

    S_min = (Q_min - b)./a;
    S_max = (Q_max - b)./a;
    for i = 1:length(S_min)
        if S_min(i) > S_max(i)
            tmp = S_min(i);
            S_min(i) = S_max(i);
            S_max(i) = tmp;
        end
    end
    [s_max, critic_joint] = min(S_max);
    critic_joint = critic_joint(1);
    s_min = max(S_min);
    if s_min > s_max || s_max < 0 || s_min > 1
        task_scaling_factor = 0;
    else
        task_scaling_factor = s_max;
    end
end