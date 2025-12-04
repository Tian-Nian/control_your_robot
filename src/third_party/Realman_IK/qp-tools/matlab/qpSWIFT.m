%% qpSWIFT - Light Weight Interior Quadratic Programming Solver
% --------------------------------------------------- 
% -------------- Installation -----------------------
%   
%   Swift_make('qpSWIFT_mex.c');
%
% ---------------------------------------------------
% ---------------------------------------------------
%	General Quadratic Program
%
%   [sol,basic_info,adv_info] = qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h)
%			or
%   [sol,basic_info,adv_info] = qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts)
%
%        minimize    0.5*x'Px + c'x
%        subject to  Ax = b
%                    Gx <= h
%
% ---------------------------------------------------
%   Only Inequality Constrained Quadratic Program
%
%  	[sol,basic_info,adv_info] = qpSWIFT(sparse(P),c,sparse(G),h)
%			or
%  	[sol,basic_info,adv_info] = qpSWIFT(sparse(P),c,sparse(G),h,opts)
%
%       minimize    0.5*x'Px + c'x
%       subject to  Gx <= h
%
% ---------------------------------------------------
%
%    INPUT arguments:
%
%       P is a sparse matrix of dimension (n,n)
%
%       c is a dense column vector of size n
%
%       A is a sparse matrix of size (p,n); p is number of equality constraints
%
%       b is a dense column vector of size p
%
%       G is a sparse matrix of size (m,n); m is the number of inequality constraints
%
%       h is a dense column vector of size m
%
%       Opts is a structure with the following fields
% 
%           -> MAXITER : maximum number of iterations needed
%           -> ABSTOL  : absolute tolerance
%           -> RELTOL  : relative tolerance
%           -> SIGMA   : maximum centering allowed
%           -> VERBOSE : PRINT LEVELS ||  0 -- No Print
%                                     || >0 -- Print everything
%           -> Permut  : permutation vector obtained as
% 
%           	KKT = [P A' G';
%                    A 0   0;
%                    G 0 -I];
%             Permut = amd(KKT);
%
%   Note: Options are not mandatory
%   Note: All the fields of Options are also not mandatory
%   Note: All the input Matrices should be sparse
% --------------------------------------------------
%
%	OUTPUT arguments:
%
%   sol represents the primal solution of the QP
%
%   basic_info has four fileds
%       -> Exit Flag : 0 : Optimal Solution Found
%                    : 1 : Failure in factorising KKT matrix
%                    : 2 : Maximum Number of Iterations Reached
%                    : 3 : Unknown Problem in Solver
%      -> Iterations : Number of Iterations
%      -> Setup Time : Invloves setting up QP; solving for initial guess
%      -> Solve Time : Solution Time
%
%   adv_info  has five fields
%      -> Fval       : Objective Value of the QP
%      -> KKT_Time   : Time needed to solve the KKT system of equations
%      -> LDL_Time   : Time needed to perform LDL' factorization
%      -> y          : Dual Variables 
%      -> z          : Dual Variables
%      -> s          : Primal Slack Variables
% 
% 
% Copyright (C) Abhishek Pandala [agp19@vt.edu], Yanran Ding [yanran@mit.edu], and Hae Won Park [haewonpark@kaist.ac.kr]
