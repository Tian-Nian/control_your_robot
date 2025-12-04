function [x0,s0,y0,z0,Phi] = qpSWIFT_init(n,m,p,P,c,G,h,A,b)

        if p == 0
            Phi = [P G'; G -eye(m,m)];
            X = Phi\[-c;h];
            x0 = X(1:n);
            y0 = [];
        else
            Phi = [P A' G'; A zeros(p,p) zeros(p,m);G zeros(m,p) -eye(m,m)];
            X = Phi\[-c;b;h];
            x0 = X(1:n);
            y0 = X(n+[1:p]);
        end
        bz = h;
        z = (G*x0)-bz;
        alpha_p = -min(-z);

    if alpha_p<0
        s0 = -z;
    else
        s0 = -z+(1+alpha_p);
    end
    
    alpha_d = -min(z);
    if alpha_d<0
        z0 = z;
    else
        z0 = z+(1+alpha_d);
    end


end