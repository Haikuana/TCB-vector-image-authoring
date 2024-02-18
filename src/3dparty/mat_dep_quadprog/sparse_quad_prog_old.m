function [x, Ax] = sparse_quad_prog(A, Aderivate, b, Aeq, beq, lb, ub, X0)
    [m, n] = size(A);
    f = zeros(6*n, 1);
    f(1:n) = A'*b(1:m);
    f(n+1:2*n) = A'*b(m+1:2*m);
    f(2*n+1:3*n) = A'*b(2*m+1:3*m);
    f(3*n+1:4*n) = A'*b(3*m+1:4*m);
    f(4*n+1:5*n) = A'*b(4*m+1:5*m);
    f(5*n+1:6*n) = A'*b(5*m+1:6*m);
      
    C = sparse(6*m, 6*n);
    C(1:m, 1:n) = A;
    C(m+1:2*m, n+1:2*n) = A;
    C(2*m+1:3*m, 2*n+1:3*n) = A;
    C(3*m+1:4*m, 3*n+1:4*n) = A;
    C(4*m+1:5*m, 4*n+1:5*n) = A;
    C(5*m+1:6*m, 5*n+1:6*n) = A;
    H0 = C'*C;
	
	H1 = sparse(6*n,6*n);
    %position
	H1(1:n,1:n) = Aderivate;
	H1(n+1:2*n,n+1:2*n) = Aderivate;
    H1(2*n+1:3*n,2*n+1:3*n)  = Aderivate;
    %color
	H1(3*n+1:4*n,3*n+1:4*n) = zeros(n,n);
    H1(4*n+1:5*n,4*n+1:5*n) = zeros(n,n);
    H1(5*n+1:6*n,5*n+1:6*n) = zeros(n,n);
	H = H0+H1;
	
    [nrow, ncol] = size(Aeq);
    Aeq_ = sparse(nrow, 6*n);
    Aeq_(1:nrow, 1:ncol) = Aeq; 
    opts = optimoptions(@quadprog,'Algorithm','interior-point-convex','MaxIterations',1000);
    x = quadprog(H,f,[],[],Aeq_,beq,lb,ub,X0,opts);
    Ax = C*x;
end