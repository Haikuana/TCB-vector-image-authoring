function [x, Ax] = sparse_quad_prog_color(A, b) 
%mcc -W cpplib:sparse_quad_prog_color -T link:lib sparse_quad_prog_color
    [m, n] = size(A);
    ncolor = 3;
 %% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%color
    A3 = sparse(ncolor*m, ncolor*n);
    A3(1:m, 1:n) = A;
    A3(m+1:2*m, n+1:2*n) = A;
    A3(2*m+1:3*m, 2*n+1:3*n) = A;
    
    mat_a = A3'*A3;
    mat_b = A3'*b;
 
   disp("the solver starting: ");
    x = mat_a \ mat_b;
    %[x,r] = linsolve(mat_a, mat_b);
 
    Ax = A3*x;
end