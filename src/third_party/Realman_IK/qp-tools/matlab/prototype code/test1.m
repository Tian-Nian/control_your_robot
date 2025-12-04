clc;
clear all;
close all;


P = [1.2204 1.1123 -3.8935;
 1.1123 3.5821 -3.3333;
-3.8935 -3.3333 19.6174];



c = [2.7694;-1.3499;3.0349];

A = [0.7254 -0.0631 0.7147];

b = -0.2050;

G = [-0.1241 1.4090 0.6715;
    1.4897 1.4172 -1.2075];

h = [0.7172;1.6302];

% [x,s,fval,jj,x_sol,s_sol,z_sol] = customQP_PredCorr_full(P,c,G,h,A,b);

[x,s,fval,jj,x_sol,s_sol,z_sol] = qpSWIFT_matlab(P,c,G,h,A,b);

% options.MAXITER = 3;

% [sol,basic_info,adv_info] = qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,options);
