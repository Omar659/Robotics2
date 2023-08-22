function q_rel = abs2rel(q_abs)
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
    n = length(q_abs);
    
    % Create a matrix of zeros
    T = zeros(n);
    
    % Fill the lower triangular part with ones
    T(tril(true(n))) = 1;
    
    % Inverse of rel to abs matrix
    T = inv(T);
    disp(T)

    q_rel = T*q_abs;   
end