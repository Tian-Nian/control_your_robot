function [x,s,fval,itr,x_sol,s_sol,z_sol] = qpSWIFT_matlab(P,c,G,h,A,b,sigma_d,x0,s0,z0)
%[x,itr,fval]= customQP_PredCorr(P,c,G,h,A,b)
%   0.5x'*P*x+c'x
%       Gx-h<=0
%       Ax-b=0

[n,m,p,P,c,G,h,A,b] = qpSWIFT_checkinputs(P,c,G,h,A,b);

if nargin<7
    sigma_d = 0;
end

sigma = 100;

if nargin<8
    [x0,s0,y0,z0,Phi] = qpSWIFT_init(n,m,p,P,c,G,h,A,b);
end


x = x0;
if p ~= 0
    y = y0;
end
z = z0;
s = s0;


x_sol = x0;
s_sol = s0;
z_sol = z0;

for itr = 0:100


    %%% ----Compute Residuals------ %%%
    if p==0
        rx = P*x+G'*z+c;% optimality residual (Dual)
        rz = s+G*x-h;%ineq residual (Primal)
    else
        rx = P*x+A'*y+G'*z+c;% optimality residual (Dual)
        ry = A*x-b;%eq residual (Primal)
        rz = s+G*x-h;%ineq residual (Primal)
    end
    %%% ----Compute Residuals------ %%%


    %%% ----Check Exit Conditions------ %%%
    if p == 0
        if norm([rx;rz])<1e-6 && s'*z/m<=1e-6 %(Barrier convergence tolerance)
            break;
        end
    else
        if norm([rx;ry;rz])<1e-6 && s'*z/m<=1e-6
            break;
        end
    end
    %%% ----Check Exit Conditions------ %%%



    W = diag(sqrt(s).*(1./sqrt(z)));
    lambda = sqrt(s).*sqrt(z);
    mu = lambda'*lambda/m;
    WTW = my_Trans_multiply_diag(W);
    inv_W_WT = my_inverse_diag_matrix(W.^2);


    if sigma>sigma_d
        ds = -lambda.*lambda;
        Wlambda_dia_ds = W'*diag(1./lambda)*ds;

            if p == 0
%                 Phi = [P G'; G -eye(m,m)];
                Phi(n+[1:m],n+[1:m]) = -WTW;
                Delta = Phi\[-rx;-rz-Wlambda_dia_ds];%my_sol_linear(nn,Ai,Ap,iA,Phi,[-rx;-rz-Wlambda_dia_ds],Permu);
                    if condest(Phi)>1e10
                        aa=1;
                    end
                Delta_z = Delta(n+[1:m]);
            else
%                 Phi = [P A' G'; A zeros(p,p) zeros(p,m);G zeros(m,p) -WTW];
                Phi(n+p+[1:m],n+p+[1:m]) = -WTW;
                Delta = Phi\[-rx;-ry;-rz-Wlambda_dia_ds];
                Delta_z = Delta(n+p+[1:m]);
            end

        Delta_s = Wlambda_dia_ds-WTW*Delta_z;

        alpha_p = findsteplength(s,Delta_s);

        alpha_d = findsteplength(z,Delta_z);

        rho = (s+alpha_p*Delta_s)'*(z+alpha_d*Delta_z)/(s'*z);
        sigma = max(sigma_d,min(1,rho)^3    );
        ds = -lambda.*lambda-(my_inverse_diag_matrix(W)*Delta_s).*(W*Delta_z)+sigma*mu*ones(m,1);
    else
        sigma = sigma_d;
        ds = -lambda.*lambda+sigma*mu*ones(m,1);
    end

    Wlambda_dia_ds = W'*diag(1./lambda)*ds;

    if  p == 0
        Phi(n+[1:m],n+[1:m]) = -WTW;
        Delta = Phi\[-rx;-rz-Wlambda_dia_ds];%my_sol_linear(nn,Ai,Ap,iA,Phi,[-rx;-rz-Wlambda_dia_ds],Permu);

        Delta_x = Delta(1:n);
        Delta_z = Delta(n+[1:m]);
    else

%         Phi = [P A' G'; A zeros(p,p) zeros(p,m);G zeros(m,p) -WTW];
        Phi(n+p+[1:m],n+p+[1:m]) = -WTW;
        Delta = Phi\[-rx;-ry;-rz-Wlambda_dia_ds];

        Delta_x = Delta(1:n);
        Delta_y = Delta(n+[1:p]);
        Delta_z = Delta(n+p+[1:m]);
    end

      Delta_s = Wlambda_dia_ds-WTW*Delta_z;

      alpha_p = findsteplength(s,Delta_s);
      alpha_d = findsteplength(z,Delta_z);

      alpha_p = min([0.99*alpha_p,1]);
      alpha_d = min([0.99*alpha_d,1]);

      if p ~= 0
         disp(['It: ', num2str(itr), ' || pcost: ', num2str(0.5*x'*P*x+c'*x), ' || rx: ', num2str(norm(rx)), ' || ry: ', num2str(norm(ry)), ' || rz: ', num2str(norm(rz)), ' || mu: ', num2str(mu),  ' || Primal Step Size : ', num2str(alpha_p), ' || Dual Step Size : ', num2str(alpha_d)])
      else
         disp(['It: ', num2str(itr), ' || pcost: ', num2str(0.5*x'*P*x+c'*x), ' || rx: ', num2str(norm(rx)), ' || rz: ', num2str(norm(rz)), ' || mu: ', num2str(mu),  ' || Primal Step Size : ', num2str(alpha_p), ' || Dual Step Size : ', num2str(alpha_d)])
      end

      x = x+alpha_p*Delta_x;
      x_sol = [x_sol x];

      if p ~= 0
          y = y+alpha_d*Delta_y;
      end

      s = s+alpha_p*Delta_s;
      z = z+alpha_d*Delta_z;
      s_sol = [s_sol s];
      z_sol = [z_sol z];

end

norm_rx = norm(rx);

if p == 0
  norm_ry = norm(rz);
else
  norm_ry = norm(ry);
end

fval = 0.5*x'*P*x+c'*x;
no_Ineq_vio = sum(G*x-h>1e-6);
