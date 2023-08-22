function [q0, H] = H_obs(q, a, b)
    % H_obs - maximize the minimum distance to Cartesian obstacles
    %
    % sintax: [q0, H] = H_man(q, a, b)
    %
    % input:
    %   q - joints position
    %   a - c x 2 robot control points, where c is the number of control
    %       points
    %   b - o x 2 obstacles, where o is the number of obstacles
    %
    % output:
    %   q0 - max of the min distance to Cartesian obstacles
    %   H - value of the function H_man
    d = zeros(size(a, 1),size(b, 1));
    for i = 1:size(a, 1)
        for j = 1:size(b, 1)
            d(i, j) = norm(a(i, :)' - b(j, :)').^2;
        end
    end
    H = min(min(d));
%     q0 = jacobian(H, q)';
    q0 = 0;
end
