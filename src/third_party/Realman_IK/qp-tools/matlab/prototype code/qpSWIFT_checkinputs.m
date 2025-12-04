function [n,m,p,P,c,G,h,A,b] = qpSWIFT_checkinputs(P,c,G,h,A,b)

    if ~issymmetric(P)
        disp('Cost Function Quadratic term is not symmetric');
        disp('Working on symmetric part of Cost function;P = (P + PT)/2');
        P = (P + P')./2;
    end
    
    [p1,p2] = size(P);
    [g1,g2] = size(G);
    [a1,a2] = size(A);
    
    [c1,c2] = size(c);
    [b1,b2] = size(b);
    [h1,h2] = size(h);
    
    if (p1 ~= p2) 
        disp('P is not a square matrix');
    end
    
    n = p1;
    m = g1;
    p = a1;
    
    if (g2 ~= n)
        disp('Number of Columns of G are not consistent');
    end
    
    if (~isempty(A))
        if (a2 ~= n)
            disp('Number of Columns of A are not consistent');
        end
    end
    
    if (p1 ~= length(c))
        disp('Dimensions of P and c are not consistent');
    end
    
    if (g1 ~= length(h))
        disp('Dimensions of G and h are not consistent');
    end
    
    if (~isempty(A) || ~isempty(A))
        if (a1 ~= length(b))
            disp('Dimensions of A and b are not consistent');
        end
    end
    
    
    if (c2 ~= 1)
        disp('c is not a column vector');
        disp('Making it a column vector');
        c = c';
    end
        
    if (h2 ~= 1)
        disp('h is not a column vector');
        disp('Making it a column vector');
        h = h';
    end
    
    if (~isempty(b))
        if (b2 ~= 1)
            disp('b is not a column vector');
            disp('Making it a column vector');
            b = b';
        end
    end
        
    
end