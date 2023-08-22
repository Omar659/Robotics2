function [q0, H] = H_man(q, J)
    % H_man - maximize the "distance" from singularities
    %
    % sintax: [q0, H] = H_man(q, J)
    %
    % input:
    %   q - joints position
    %   J - Jacobian
    %
    % output:
    %   q0 - max distance from singularities
    %   H - value of the function H_man
    disp("Controlla le dimensioni di J miraccomando")
    H = sqrt(det(J*J'));
    q0 = jacobian(H, q)';
end