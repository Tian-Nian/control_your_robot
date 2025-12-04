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

%        if nA ==0
%            Phi = [P G'; G -eye(nz,nz)];
%         else
%             Phi = [P A' G'; A zeros(nA,nA) zeros(nA,nG);G zeros(nG,nA) -eye(nz,nz)];
%         end
        
%        if nA==0
%            vec0 = Phi\[-c;h];
if nargin<8
        if nA==0
        %    L = chol((P+G'*G),'lower');
%             Phi = [P G'; G -eye(nz,nz)];
%             vec0 = ldlsparse(sparse(Phi),p,[-c;h]);
%             x0 = vec0(1:nx);
%             L = TEST_Cholesky((P+G'*G));
%             x0 = my_backward_solve(L',my_forward_solve(L,-c+G'*h));
            Phi = [P G'; G -eye(nz,nz)];
     %     Phi(nx+[1:nz],nx+[1:nz]) = -WTW;
           
            X = Phi\[-c;h];%my_sol_linear(nn,Ai,Ap,iA,Phi,[-c;h],Permu);
            x0 = X(1:nx);
        else
            Phi = [P A' G'; A zeros(nA,nA) zeros(nA,nG);G zeros(nG,nA) -eye(nz,nz)];
            X = Phi\[-c;b;h];
            x0 = X(1:nx);
            y0 = X(nx+[1:ny]);

%             L = chol((P+G'*G),'lower');
%             GTH = G'*h;
%             LAT = my_forward_solve(L,A');
%             LL = chol(LAT'*LAT,'lower');
%             y0 = my_backward_solve(LL',my_forward_solve(LL,A*my_backward_solve(L',my_forward_solve(L,-c+GTH))-b));
%             x0 = my_backward_solve(L',my_forward_solve(L,-c+GTH-A'*y0));
        end

            %x0 = L'\(L\(-c+G'*h));
%            x0 = vec0(1:nx);
%         else
%             vec0 = Phi\[-c;b;h];
%             x0 = vec0(1:nx);
%             y0 = vec0(nx+[1:ny]);
%         end
        
        bz = h;
    z = (G*x0)-bz;
%     x0 = vec0(1:nx);
%     y0 = vec0(nx+[1:ny]);
%     z = h-G*x0;
    alpha_p = -min(-z);
    %alpha_p = -z+alpha*ones(size(z,1),1);
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


% x0  = ones(nx,1);
% s0 = ones(nz,1);
% z0 = ones(nz,1);
% 
% 
% 
% %         rx = P*x0+G'*z0+c;
% %         rz = s0+G*x0-h;
%          Phi0 = [P zeros(nx,nz) G'; G eye(nz,nz) zeros(nz,nz); zeros(nz,nx) diag(s0) diag(z0)];
%          bb = [P*x+c+G'*z;G*x-h+s;s0.*z0];
%    XX = Phi0\bb     ;
%    dx = XX(1:nx);
%    ds = XX(nx+[1:nz]);
%    dz = XX(nx+nz+[1:nz]);
% x0 = x0+dx   ;
% s__ = abs(s+ds);
% s__(s__<1) = 1;
% s0 = s__;
% z__ = abs(z+dz);
% z__(z__<1) = 1;
% z0 = z__;

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
inv_W_WT = my_inverse_diag_matrix(W.^2);

if sigma>sigma_d
    ds = -lambda.*lambda;
    Wlambda_dia_ds = W'*diag(1./lambda)*ds;
    
       if nA==0
%                L = TEST_Cholesky(P+G'*inv_W_WT*G);
%                 bz = (-rz-Wlambda_dia_ds);
%                 Delta_x = my_backward_solve(L',my_forward_solve(L,[-rx+G'*inv_W_WT*bz]));
%                 
                 %Phi = [P G'; G  -WTW];
                 Phi(nx+[1:nz],nx+[1:nz]) = -WTW;
%            sPhi = sparse(Phi);
%            Permut = symamd(sPhi);
%            [x, fl] = ldlsparse (sPhi,Permut,[-rx;-rz-Wlambda_dia_ds]);
                 
                Delta = Phi\[-rx;-rz-Wlambda_dia_ds];%my_sol_linear(nn,Ai,Ap,iA,Phi,[-rx;-rz-Wlambda_dia_ds],Permu);
                if condest(Phi)>1e10
                    aa=1;
                end
% % % %             Phi = sparse([P G'; G  -WTW]);
% % % %             Delta = ldlsparse(sparse(Phi),p,[-rx;-rz-Wlambda_dia_ds]);
                Delta_z = Delta(nx+[1:nz]);

                %Delta_x = L'\(L\[-rx+G'*inv_W_WT*bz]);
%            Delta_z = inv_W_WT*(G*Delta_x-bz);
%                Phi = [P G'; G  -WTW];
%                Delta = Phi\[-rx;-rz-Wlambda_dia_ds];
%            Delta_z = Delta(nx+[1:nz]);

       else
%             L = chol((P+G'*inv_W_WT*G),'lower');
%             bz = (-rz-Wlambda_dia_ds);
%             LAT = my_forward_solve(L,A');
%             LL = chol(LAT'*LAT,'lower');
%             Delta_y = my_backward_solve(LL',my_forward_solve(LL,A*my_backward_solve(L',my_forward_solve(L,-rx+G'*inv_W_WT*bz))+ry));
%             Delta_x = my_backward_solve(L',my_forward_solve(L,-rx+G'*inv_W_WT*bz-A'*Delta_y));
%             Delta_z = inv_W_WT*(G*Delta_x-bz);
           Phi = [P A' G'; A zeros(nA,nA) zeros(nA,nG);G zeros(nG,nA) -WTW];
           Delta = Phi\[-rx;-ry;-rz-Wlambda_dia_ds];
%            sPhi = sparse(Phi);
%            Permut = symamd(sPhi);
%            [Delta, fl] = ldlsparse (sPhi,Permut,[-rx;-ry;-rz-Wlambda_dia_ds]);
           %[L, D, Parent, fl] = ldlsparse (sparse(Phi), Permut);
           Delta_z = Delta(nx+ny+[1:nz]);

        end
%        Delta_x = Delta(1:nx);
%        Delta_y = Delta(nx+[1:ny]);
%    Delta_z = Delta(nx+ny+[1:nz]);
    Delta_s = Wlambda_dia_ds-WTW*Delta_z;
    if 0
        n = 1001;
        alpha_ = 1/n:1/n:1;
        for ii=1:length(alpha_)
                if any([s+alpha_(ii)*(Delta_s);z+alpha_(ii)*Delta_z]<0)
                    if ii==1
                        alpha = 0;
                    else
                        alpha = alpha_(ii-1);
                    end
                    break;
                else
                    alpha = 1;
                end
        end
    else
        Test_s = s./Delta_s;
         Test_s(s./Delta_s>0) = -1000;
         [~,idx] = max(Test_s);
         alpha_p = -Test_s(idx);

         Test_z = z./Delta_z;
         Test_z(z./Delta_z>0) = -1000;
         [~,idx] = max(Test_z);
         alpha_d = -Test_z(idx);
        Test_s = [[s;z]./[Delta_s;Delta_z]];
        Test_s([s;z]./[Delta_s;Delta_z]>0) = -1000;
         [~,idx] = max(Test_s);
         alpha = -Test_s(idx);
%        alpha_p = 1;
        
%         while any([s+alpha_p*(Delta_s)]<0)
%             alpha_p = t*alpha_p;
%         end
%         alpha_d = 1;
%         while any([z+alpha_d*(Delta_z)]<0)
%             alpha_d = t*alpha_d;
%         end        
    end
    rho = (s+alpha*Delta_s)'*(z+alpha*Delta_z)/(s'*z);
    sigma = max(sigma_d,min(1,rho)^3);
    ds = -lambda.*lambda-(my_inverse_diag_matrix(W)*Delta_s).*(W*Delta_z)+sigma*mu*ones(ns,1);
else
    sigma = sigma_d;
    ds = -lambda.*lambda+sigma*mu*ones(ns,1);
end

Wlambda_dia_ds = W'*diag(1./lambda)*ds;
   if nA==0
        
         %Phi = [P G'; G -WTW];
        %Delta = Phi\[-rx;-rz-Wlambda_dia_ds];
         Phi(nx+[1:nz],nx+[1:nz]) = -WTW;
        Delta = Phi\[-rx;-rz-Wlambda_dia_ds];%my_sol_linear(nn,Ai,Ap,iA,Phi,[-rx;-rz-Wlambda_dia_ds],Permu);

       Delta_x = Delta(1:nx);
       Delta_z = Delta(nx+[1:nz]);
% % % %          Delta = ldlsparse(sparse(Phi),p,[-rx;-rz-Wlambda_dia_ds]);
       

% % %         L = TEST_Cholesky(P+G'*inv_W_WT*G);
% % %         bz = (-rz-Wlambda_dia_ds);
% % %         Delta_x = my_backward_solve(L',my_forward_solve(L,[-rx+G'*inv_W_WT*bz]));
% % % 
% % % %                Delta_x = L'\(L\[-rx+G'*inv_W_WT*bz]);
% % %         Delta_z = inv_W_WT*(G*Delta_x-bz);
       %Delta_x = Delta(1:nx);
       %Delta_z = Delta(nx+[1:nz]);
        
   else
%             L = chol((P+G'*inv_W_WT*G),'lower');
%             bz = (-rz-Wlambda_dia_ds);
%             LAT = my_forward_solve(L,A');
%             LL = chol(LAT'*LAT,'lower');
%             Delta_y = my_backward_solve(LL',my_forward_solve(LL,A*my_backward_solve(L',my_forward_solve(L,-rx+G'*inv_W_WT*bz))+ry));
%             Delta_x = my_backward_solve(L',my_forward_solve(L,-rx+G'*inv_W_WT*bz-A'*Delta_y));
%             Delta_z = inv_W_WT*(G*Delta_x-bz);
        
       
        Phi = [P A' G'; A zeros(nA,nA) zeros(nA,nG);G zeros(nG,nA) -WTW];
        Delta = Phi\[-rx;-ry;-rz-Wlambda_dia_ds];
%        [Delta, fl] = ldlsparse (sPhi,Permut,[-rx;-ry;-rz-Wlambda_dia_ds]);
        
       Delta_x = Delta(1:nx);
       Delta_y = Delta(nx+[1:ny]);
       Delta_z = Delta(nx+ny+[1:nz]);
    end

%    Delta_z = Delta(nx+ny+[1:nz]);
    Delta_s = Wlambda_dia_ds-WTW*Delta_z;


% Delta_x = Delta(1:nx);
% Delta_y = Delta(nx+[1:ny]);
% Delta_z = Delta(nx+ny+[1:nz]);
% Delta_s = Wlambda_dia_ds-WTW*Delta_z;


if 0
    n = 1001;
    alpha_ = 1/n:1/n:1;

    for ii=1:length(alpha_)
    %        if any([lambda+alpha_(ii)/0.99*(W'\Delta_s);lambda+alpha_(ii)/0.99*W*Delta_z]<0)
            if any([s+alpha_(ii)*(Delta_s);z+alpha_(ii)*Delta_z]<0)
                if ii==1
                    alpha = 0;
                else
                    alpha = alpha_(ii-1);
                end
                break;
            else
                alpha = 1;
            end
    end
else
    
        Test_s = s./Delta_s;
         Test_s(s./Delta_s>0) = -1000;
         [~,idx] = max(Test_s);
         alpha_p = -Test_s(idx);

         Test_z = z./Delta_z;
         Test_z(z./Delta_z>0) = -1000;
         [~,idx] = max(Test_z);
        Test_s = [[s;z]./[Delta_s;Delta_z]];
        Test_s([s;z]./[Delta_s;Delta_z]>0) = -1000;
         [~,idx] = max(Test_s);
         alpha = -Test_s(idx);         
         %alpha_d = -Test_z(idx);
        alpha = min([0.99*alpha,1]);
        %alpha_d = min([0.99*alpha_d,1]);
        
    
%     t = 0.75;
%     alpha_p = 1;
% 
%     while any([s+alpha_p*(Delta_s)]<0)
%         alpha_p = t*alpha_p;
%     end
%     alpha_d = 1;
% 
%     while any([z+alpha_d*Delta_z]<0)
%         alpha_d = t*alpha_d;
%     end    
end
% disp(['It: ', num2str(jj), ' || pcost: ', num2str(0.5*x'*P*x+c'*x), ' || rx: ', num2str(norm(rx)), ' || ry: ', num2str(norm(ry)), ' || rz: ', num2str(norm(rz)), ' || mu: ', num2str(mu),  ' || Primal Step Size : ', num2str(alpha_p), ' || Dual Step Size : ', num2str(alpha_d)])
x = x+alpha*Delta_x;
x_sol = [x_sol x];
if nA==0    
else
    y = y+alpha*Delta_y;
end
s = s+alpha*Delta_s;
z = z+alpha*Delta_z;
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