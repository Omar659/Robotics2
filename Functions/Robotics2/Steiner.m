function I = Steiner(I_c, m, r)
    % Steiner - Steiner theorem
    %
    % sintax: I = Steiner(I_c, m, r)
    %
    % input:
    %   I_c - inertia matrix around COM
    %   m - mass of the link
    %   r - vector to point of rotation from COM 
    %
    % output:
    %   I - inertia matrix around the new reference frame
    
    I = I_c + m*skew_symmetric3(r)'*skew_symmetric3(r);
end