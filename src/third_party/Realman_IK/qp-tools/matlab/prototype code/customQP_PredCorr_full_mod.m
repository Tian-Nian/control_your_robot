function [x,s,fval,jj,x_sol,s_sol,z_sol] = customQP_PredCorr_full(P,c,G,h,A,b,sigma_d,x0,s0,z0)
%[x,jj,fval]= customQP_PredCorr(P,c,G,h,A,b)nn
%0.5x'*P*x+c'x
%Gx-h<=0
%Ax-b=0
% P = Data.H;
% c = Data.g;
% G = Data.P;
% h = Data.h;
% A = Data.C;
% b = Data.b;

P = 0.5*(P+P');
n_states = 18;%number of states
n_input = 12;%number of input    


nA = size(A,1);
mA = size(A,2);
nG = size(G,1);
mG = size(G,2);
nx = size(P,1);
ny = size(A,1);
% nz = size(G,1);
% ns = nz;

nx = size(P,1);
nz = size(G,1);
ns = size(G,1);
if nargin<7
    sigma_d = 0;
end
sigma = 100;


if nargin<8
        if nA==0
            Phi = [P G'; G -eye(nz,nz)];
            X = Phi\[-c;h];%my_sol_linear(nn,Ai,Ap,iA,Phi,[-c;h],Permu);
            x0 = X(1:nx);
        else
            Phi = [P A' G'; A zeros(nA,nA) zeros(nA,nG);G zeros(nG,nA) -eye(nz,nz)];
            X = Phi\[-c;b;h];
            x0 = X(1:nx);
            y0 = X(nx+[1:ny]);
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
Phi = [P G'; G -eye(nz,nz)];
m = length(z0);
x = x0;
if nA==0
else
    y = y0;
end
z = z0;
s = s0;


x_sol = x0;
s_sol = s0;
z_sol = z0;
for jj=0:100
    

    if nA==0
        rx = P*x+G'*z+c;% optimality residual (Dual)
        rz = s+G*x-h;%ineq residual (Primal)
    else
        rx = P*x+A'*y+G'*z+c;% optimality residual (Dual)
        ry = A*x-b;%eq residual (Primal)
        rz = s+G*x-h;%ineq residual (Primal)
    end
%norm([rx;ry;rz])
if nA==0
    if norm([rx;rz])<1e-6 && s'*z/nz<=1e-6 %(Barrier convergence tolerance)
        break;
    end    
else
    if norm([rx;ry;rz])<1e-6&& s'*z/nz<=1e-6
        break;
    end    
end

W = diag(sqrt(s).*(1./sqrt(z)));
%W = eye(size(s,1),size(s,1)); 
lambda = sqrt(s).*sqrt(z);
mu = lambda'*lambda/m;


% dx = -rx;
% dy = -ry;
% dz = -rz;
WTW = my_Trans_multiply_diag(W);
% inv_W_WT = my_inverse_diag_matrix(W.^2);

if sigma>sigma_d
    ds = -lambda.*lambda;
    Wlambda_dia_ds = W'*diag(1./lambda)*ds;
    
       if nA==0
                 Phi(nx+[1:nz],nx+[1:nz]) = -WTW;
                Delta = Phi\[-rx;-rz-Wlambda_dia_ds];%my_sol_linear(nn,Ai,Ap,iA,Phi,[-rx;-rz-Wlambda_dia_ds],Permu);
                if condest(Phi)>1e10
                    aa=1;
                end
                Delta_z = Delta(nx+[1:nz]);
       else
           Phi = [P A' G'; A zeros(nA,nA) zeros(nA,nG);G zeros(nG,nA) -WTW];
           Delta = Phi\[-rx;-ry;-rz-Wlambda_dia_ds];
           Delta_z = Delta(nx+ny+[1:nz]);

        end
    Delta_s = Wlambda_dia_ds-WTW*Delta_z;

        Test_s = s./Delta_s;
         Test_s(s./Delta_s>0) = -1000;
         [~,idx] = max(Test_s);
         alpha_p = -Test_s(idx);

         Test_z = z./Delta_z;
         Test_z(z./Delta_z>0) = -1000;
         [~,idx] = max(Test_z);
         alpha_d = -Test_z(idx);

         
         
    rho = (s+alpha_p*Delta_s)'*(z+alpha_d*Delta_z)/(s'*z);
    
    if (jj==0)
            disp(['s : ',num2str(s(1),10),';',num2str(s(2),10)]);
            disp(['z : ',num2str(z(1),10),';',num2str(z(2),10)]);
            disp(['delta_s : ',num2str(Delta_s(1),10),';',num2str(Delta_s(2),10)]);
            disp(['delta_z : ',num2str(Delta_z(1),10),';',num2str(Delta_z(2),10)]);
            disp(['alpha_p : ',num2str(alpha_p,10)]);
            disp(['alpha_d : ',num2str(alpha_d,10)]);
            disp(['rho     : ',num2str(rho,10)]);
    end
    
    
    sigma = max(sigma_d,min(1,rho)^3);
    ds = -lambda.*lambda-(my_inverse_diag_matrix(W)*Delta_s).*(W*Delta_z)+sigma*mu*ones(ns,1);
else
    sigma = sigma_d;
    ds = -lambda.*lambda+sigma*mu*ones(ns,1);
end

Wlambda_dia_ds = W'*diag(1./lambda)*ds;
   if nA==0
        
        
         Phi(nx+[1:nz],nx+[1:nz]) = -WTW;
        Delta = Phi\[-rx;-rz-Wlambda_dia_ds];%my_sol_linear(nn,Ai,Ap,iA,Phi,[-rx;-rz-Wlambda_dia_ds],Permu);

       Delta_x = Delta(1:nx);
       Delta_z = Delta(nx+[1:nz]);       
   else

        Phi = [P A' G'; A zeros(nA,nA) zeros(nA,nG);G zeros(nG,nA) -WTW];
        Delta = Phi\[-rx;-ry;-rz-Wlambda_dia_ds];
        
       Delta_x = Delta(1:nx);
       Delta_y = Delta(nx+[1:ny]);
       Delta_z = Delta(nx+ny+[1:nz]);
    end

    Delta_s = Wlambda_dia_ds-WTW*Delta_z;
        
        Test_s = s./Delta_s;
         Test_s(s./Delta_s>0) = -1000;
         [~,idx] = max(Test_s);
         alpha_p = -Test_s(idx);

         Test_z = z./Delta_z;
         Test_z(z./Delta_z>0) = -1000;
         [~,idx] = max(Test_z);
         alpha_d = -Test_z(idx);
         
        
         
        alpha_p = min([0.99*alpha_p,1]);
        alpha_d = min([0.99*alpha_d,1]);
        

% disp(['It: ', num2str(jj), ' || pcost: ', num2str(0.5*x'*P*x+c'*x), ' || rx: ', num2str(norm(rx)), ' || ry: ', num2str(norm(ry)), ' || rz: ', num2str(norm(rz)), ' || mu: ', num2str(mu),  ' || Primal Step Size : ', num2str(alpha_p), ' || Dual Step Size : ', num2str(alpha_d)])
x = x+alpha_p*Delta_x;
x_sol = [x_sol x];
if nA==0    
else
    y = y+alpha_d*Delta_y;
end
s = s+alpha_p*Delta_s;
z = z+alpha_d*Delta_z;
s_sol = [s_sol s];
z_sol = [z_sol z];
end
norm_rx = norm(rx);
if nA==0
    norm_ry = norm(rz);
else
    norm_ry = norm(ry);
end

fval = 0.5*x'*P*x+c'*x;
no_Ineq_vio = sum(G*x-h>1e-6);

