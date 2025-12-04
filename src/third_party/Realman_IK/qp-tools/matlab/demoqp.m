%%% Sample Quadratic Program for qpSWIFT

clc;
clear all;
close all;


%%%% Solver Options
%%% For information about Solver options please refer to qpSWIFT
%%% documentation or type help qpSWIFT in the matlab command prompt
opts.VERBOSE = 1;


%%%% Cost Function
P = [5 1 0;1 2 1;0 1 4];
c = [1;2;1];


%%%% Equality Constraints
A = [1 -2 1];
b = 3;


%%%% Inequality Constraints
G = [-4 -4 0;0 0 -1];
h = [-1;-1];



%%% Equality Constrained Quadratic Program
fprintf("-----Equality Constrained Quadratic Program-----\n\n");
[soleq,basic_infoeq,adv_infoeq] = qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);


%%% Inequality Constrained Quadratic Program
fprintf("-----Inequality Constrained Quadratic Program-----\n\n");
[sol,basic_info,adv_info] = qpSWIFT(sparse(P),c,sparse(G),h,opts);
