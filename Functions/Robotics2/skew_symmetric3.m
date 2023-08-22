function S = skew_symmetric3(r)
    % skew_symmetric - Compute the skew symmetric matrix of r
    %
    % S = skew_symmetric(r)
    %
    % input:
    %   r - A 3x1 vector
    %
    % output:
    %   S - The skew symmetric matrix 3x3

    S = [ 0    -r(3)  r(2);
          r(3)  0    -r(1);
         -r(2)  r(1)  0];
end