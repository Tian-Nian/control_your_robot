clc;
clear;
close all;


load('Matrix.mat');

sigma_d = 0.0;
Phi = [P A' G'; A zeros(p ,m + p);G zeros(m,p) -eye(m,m)];

Permut = amd(Phi);


opts.MAXITER = 25;
opts.ABSTOL = 1e-6;
opts.RELTOL = 1e-6;
opts.PERMUT = Permut;
opts.VERBOSE = 0;
 


for i = 1:1e3
[X,basic_info,adv_info]  = qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);

[X1,basic_info1,adv_info1] = qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);
    
end



Phi = [P G';G -eye(m,m)];
Permut = amd(Phi);
opts.PERMUT = Permut;


for i = 1:1e3
    [X2,basic_info2,adv_info2] = qpSWIFT(sparse(P),c,sparse(G),h);

    [X3,basic_info3,adv_info3] = qpSWIFT(sparse(P),c,sparse(G),h,opts);
end

fprintf("Basic Test Passed\n");