function q_abs = rel2abs(q_rel)
    % rel2abs - change angle from relative to absolute
    %
    % sintax: q_abs = rel2abs(q_rel)
    %
    % input:
    %   q_rel - relative joint angles
    %
    % output:
    %   q_abs - absolute joint angles

    % Specify the size of the matrix
    n = length(q_rel);
    
    % Create a matrix of zeros
    T = zeros(n);
    
    % Fill the lower triangular part with ones
    T(tril(true(n))) = 1;
    
    q_abs = T*q_rel;   
end