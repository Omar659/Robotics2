function I = inertia_matrix(n)
    % inertia_matrix - compute the symbolic inertia matrix
    %
    % sintax: I = inertia_matrix(n)
    %
    % input:
    %   n - number of joints
    %
    % output:
    %   I - The inertia matrices
    characters = ["x", "y", "z"];
    I = zeros(3, 3, n)*sym("a");
    for k = 1:n
        for i = 1:3
            for j = i:3
                I(i,j,k) = sym(strcat("i", characters(i), characters(j), int2str(k)), "real");
            end
        end
        I(:, :, k)=(I(:, :, k)+I(:, :, k)').*(ones(3)-eye(3)*0.5);
    end
end

