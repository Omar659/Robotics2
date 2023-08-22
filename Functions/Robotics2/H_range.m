function [q0, H] = H_range(q, q_max, q_min)
    % H_range - minimize the “distance” from the mid points of the joint ranges
    %
    % sintax: [q0, H] = H_range(q, q_max, q_min)
    %
    % input:
    %   q - joints position
    %   q_max - max range of joints
    %   q_min - min range of joints
    %
    % output:
    %   q0 - min distance from the mid points of the joint ranges
    %   H - value of the function H_range
    N = length(q);
    q_bar = (q_max + q_min)/2;
    H = 1/(2*N) * sum(((q - q_bar)./(q_max - q_min)).^2);
    q0 = -jacobian(H, q)';
end