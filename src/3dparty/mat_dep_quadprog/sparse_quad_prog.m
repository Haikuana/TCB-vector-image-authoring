function [x, Ax,Nrank,Nullb] = sparse_quad_prog(A, Afeapos,Aboundary,Afeacolor,Aderivate, b, Aeq, beq, lb, ub)
%Nrank,Jv   
%mcc -W cpplib:sparse_quad_prog -T link:lib sparse_quad_prog
    [m, n] = size(A);
    npos = 2;
    ncolor = 3;
    [nrow, ncol] = size(Aeq);
    opts = optimoptions(@quadprog,'Algorithm','interior-point-convex','MaxIterations',1000);
   %%   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%position
    fp = zeros(npos*n, 1);
    fp(1:n) = (A+Afeapos+Aboundary)'*b(1:m);
    fp(n+1:2*n) = (A+Afeapos+Aboundary)'*b(m+1:2*m);
    %basis coeficient  
    Cp = sparse(npos*m, npos*n);
    Cp(1:m, 1:n) = A+Afeapos+Aboundary;
    Cp(m+1:2*m, n+1:2*n) = A+Afeapos+Aboundary;
    H0p = Cp'*Cp;
	%derivate smooth
	H1p = sparse(npos*n,npos*n);
	H1p(1:n,1:n) = Aderivate;
	H1p(n+1:2*n,n+1:2*n) = Aderivate;
	Hp = H0p+H1p;
    %out
    Tp = (A+Afeapos+Aboundary)'*(A+Afeapos+Aboundary);
    rank_p = rank(full(Tp));
    cond_p=1;
    %cond_p = condest(Tp,1);
    %[R,Jv]=rref(full(Tp));
    Nullb(1)=0;
    NullT=null(full(Tp));
    [ro,co]=size(NullT);
    index = 1;
    for i = 1:ro
        for j = 1:co
           if(abs(NullT(i,j)) > 1e-5)
               Nullb(index)=i;
               index = index + 1;
               break;
           end
        end
    end
    
    %compute
    Aeq_p = sparse(nrow, npos*n);
    Aeq_p(1:nrow, 1:ncol) = Aeq; 
    lb_p = lb(1:npos*n);
    ub_p = ub(1:npos*n);
    xp= quadprog(Hp,fp,[],[],Aeq_p,beq,lb_p,ub_p,[],opts);
    %out
    Basis_pos = sparse(npos*m, npos*n);
    Basis_pos(1:m, 1:n) = A;
    Basis_pos(m+1:2*m, n+1:2*n) = A;
    surface_pos = Basis_pos*xp;
    %% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%color
    fc = zeros(ncolor*n, 1);
    fc(1:n) = (A+Afeacolor)'*b(2*m+1:3*m);
    fc(n+1:2*n) = (A+Afeacolor)'*b(3*m+1:4*m);
    fc(2*n+1:3*n) = (A+Afeacolor)'*b(4*m+1:5*m);
      
    Cc = sparse(ncolor*m, ncolor*n);
    Cc(1:m, 1:n) = A+Afeacolor;
    Cc(m+1:2*m, n+1:2*n) = A+Afeacolor;
    Cc(2*m+1:3*m, 2*n+1:3*n) = A+Afeacolor;
    Hc = Cc'*Cc;	
    Tc=(A+Afeacolor)'*(A+Afeacolor);
    rank_c = rank(full(Tc));
    cond_c=1;
    %cond_c = condest(Tc);
    
    lb_c = lb(npos*n+1:(npos+ncolor)*n);
    ub_c = ub(npos*n+1:(npos+ncolor)*n);
    xc= quadprog(Hc,fc,[],[],[],[],lb_c,ub_c,[],opts);
    
    Basis_col = sparse(ncolor*m, ncolor*n);
    Basis_col(1:m, 1:n) = A;
    Basis_col(m+1:2*m, n+1:2*n) = A;
    Basis_col(2*m+1:3*m, 2*n+1:3*n) = A;
    surface_col = Basis_col*xc;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%
    x = [xp;xc];
    Ax = [surface_pos;surface_col];
    %Ncond = [cond_c cond_p];
    Nrank = [rank_c cond_c rank_p cond_p n];
end