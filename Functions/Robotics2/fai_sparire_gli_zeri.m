function B = fai_sparire_gli_zeri(A)
    % fai_sparire_gli_zeri - delete the small numbers (<1e-8) because they
    % are actually 0
    %
    % sintax: B = fai_sparire_gli_zeri(A)
    %
    % input:
    %   A - a matrix
    %
    % output:
    %   B - the matrix A without zero numbers
    B=A;
    for i=1:size(A,1)
        for j=1:size(A,2)
            aij=A(i,j);
            [C,T] = coeffs(simplify(aij));
            B(i,j)=simplify(dot(round(C,8),T));
        end
    end
end
