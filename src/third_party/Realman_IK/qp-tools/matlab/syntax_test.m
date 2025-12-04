clc;
clear all;
close all;

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

%%% Number of Arguments Test




%%% Number of Input Arguments Test
assert(error_ctrl('x=qpSWIFT(1)','Please enter the correct number of input arguments'),'Argument Test Failed_1');
assert(error_ctrl('x=qpSWIFT(1,2)','Please enter the correct number of input arguments'),'Argument Test Failed_2');
assert(error_ctrl('x=qpSWIFT(1,2,3)','Please enter the correct number of input arguments'),'Argument Test Failed_3');
assert(error_ctrl('x=qpSWIFT(1,2,3,4,5,6,7,8)','Please enter the correct number of input arguments'),'Argument Test Failed_8');
assert(error_ctrl('x=qpSWIFT(1,2,3,4,5,6,7,8,9)','Please enter the correct number of input arguments'),'Argument Test Failed_9');
%%% Number of Input Arguments Test

%%% Number of Output Arguments Test
assert(error_ctrl('[x,a,b,c]=qpSWIFT(1,2,3,4,5,6,7)','The number of outputs is restricted to 3'),'Argument Test Failed_out');
%%% Number of Output Arguments Test


%%% c,h,b test
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),[c,1],sparse(A),b,sparse(G),h);'],'Dimensions of P and c do not match'),'c dim Test Failed');
assert(error_ctrl(['load("Matrix.mat");','c(1)=i;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);'],'c should be a real dense vector'),'c double Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),num2cell(c),sparse(A),b,sparse(G),h);'],'c should be a real dense vector'),'c double Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),sparse(c),sparse(A),b,sparse(G),h);'],'c should be a real dense vector'),'c dense Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),[c;c],sparse(A),b,sparse(G),h);'],'c should be a column vector'),'c vector Test Failed');


assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),[h,1]);'],'Dimensions of G and h do not match'),'h dim Test Failed');
assert(error_ctrl(['load("Matrix.mat");','h(1)=i;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);'],'h should be a real dense vector'),'h double Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),num2cell(h));'],'h should be a real dense vector'),'h double Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),sparse(h));'],'h should be a real dense vector'),'h dense Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),[h;h]);'],'h should be a column vector'),'h vector Test Failed');


assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),[b,1],sparse(G),h);'],'Dimensions of A and b do not match'),'b dim Test Failed');
assert(error_ctrl(['load("Matrix.mat");','b(1)=i;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);'],'b should be a real dense vector'),'b double Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),num2cell(b),sparse(G),h);'],'b should be a real dense vector'),'b double Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),sparse(b),sparse(G),h);'],'b should be a real dense vector'),'b dense Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),[b;b],sparse(G),h);'],'b should be a column vector'),'b vector Test Failed');


%%% P,A,G test
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(P,c,sparse(A),b,sparse(G),h);'],'P should be a real sparse matrix'),'P sparse Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(num2cell(P),c,sparse(A),b,sparse(G),h);'],'P should be a real sparse matrix'),'P double Test Failed');
assert(error_ctrl(['load("Matrix.mat");P(1)=i;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);'],'P should be a real sparse matrix'),'P double Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(A),c,sparse(A),b,sparse(G),h);'],'P should be a square matrix'),'P dim Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P(1:end-1,1:end-1)),c,sparse(A),b,sparse(G),h);'],'Dimensions of P and c do not match'),'P dim Test Failed');

assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,G,h);'],'G should be a real sparse matrix'),'G sparse Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,num2cell(G),h);'],'G should be a real sparse matrix'),'G double Test Failed');
assert(error_ctrl(['load("Matrix.mat");G(1)=i;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);'],'G should be a real sparse matrix'),'G double Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(A),h);'],'Dimensions of G and h do not match'),'G dim Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(h),h);'],'Dimensions of G and h do not match'),'G dim Test Failed');

assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,A,b,sparse(G),h);'],'A should be a real sparse matrix'),'A sparse Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,num2cell(A),b,sparse(G),h);'],'A should be a real sparse matrix'),'A double Test Failed');
assert(error_ctrl(['load("Matrix.mat");A(1)=i;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);'],'A should be a real sparse matrix'),'A double Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(G),b,sparse(G),h);'],'Dimensions of A and b do not match'),'A dim Test Failed');
assert(error_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(b),b,sparse(G),h);'],'Dimensions of A and b do not match'),'A dim Test Failed');


%%% opts syntax test
assert(error_ctrl(['load("Matrix.mat");opts.MAXITER=-2;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Iteration Number must be within 0 and 200'),'opts maxit Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.maxiter=-2;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Iteration Number must be within 0 and 200'),'opts maxit Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.maxiter=201;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Iteration Number must be within 0 and 200'),'opts maxit Test Failed');

assert(error_ctrl(['load("Matrix.mat");opts.ABSTOL=-0.1;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Absolute Tolerance must be within 0 and 1'),'opts abstol Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.abstol=-0.1;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Absolute Tolerance must be within 0 and 1'),'opts abstol Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.abstol=1.1;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Absolute Tolerance must be within 0 and 1'),'opts abstol Test Failed');

assert(error_ctrl(['load("Matrix.mat");opts.RELTOL=-0.1;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Relative Tolerance must be within 0 and 1'),'opts reltol Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.reltol=-0.1;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Relative Tolerance must be within 0 and 1'),'opts reltol Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.reltol=1.1;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Relative Tolerance must be within 0 and 1'),'opts reltol Test Failed');

assert(error_ctrl(['load("Matrix.mat");opts.SIGMA=-0.1;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Sigma must be positive'),'opts sigma Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.sigma=-0.1;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Sigma must be positive'),'opts sigma Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.verbose=-1;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Verbose must be positive'),'opts verbose Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.VERBOSE=-1;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Verbose must be positive'),'opts verbose Test Failed');


assert(error_ctrl(['load("Matrix.mat");opts.permut=i;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Permut should be a real dense vector'),'opts permut double complex Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.PERMUT=i;','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Permut should be a real dense vector'),'opts permut double complex Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.permut=sparse([2;3]);','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Permut should be a real dense vector'),'opts permut double Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.permut=num2cell([2;3]);','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Permut should be a real dense vector'),'opts permut double Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.permut=rand(2);','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Permut should be a column vector'),'opts permut dim Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.permut=rand(size(P,1)+size(A,1)+size(G,1)+1,1);','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'The dimensions of Permutation vector are inconsistent'),'opts permut dim Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.permut=rand(size(P,1)+size(A,1)+size(G,1)-1,1);','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'The dimensions of Permutation vector are inconsistent'),'opts permut dim Test Failed');
assert(error_ctrl(['load("Matrix.mat");opts.permut=rand(size(P,1)+size(A,1)+size(G,1),1);','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h,opts);'],'Not a Valid Permutation Vector'),'opts permut dim Test Failed');


%%% Output arguments test
assert(output_ctrl(['load("Matrix.mat");','sol=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);if(~isreal(sol) || (size(sol,1)==1) || (size(sol,1) ~= n)) res = false;else res=true ;end']),'sol Test Failed');

assert(output_ctrl(['load("Matrix.mat");','[sol,binfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);if(~isreal(sol) || (size(sol,1)==1) || (size(sol,1) ~= n)) res = false;else res=true ;end']),'binfo sol Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol=binfo.Setup_Time;if(~isreal(sol) || (size(sol,1)~=1) || (length(sol) > 1)) res = false;else res=true ;end']),'binfo Setup_Time Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol=binfo.Solve_Time;if(~isreal(sol) || (size(sol,1)~=1) || (length(sol) > 1)) res = false;else res=true ;end']),'binfo Solve_Time Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol=binfo.ExitFlag;if(~isreal(sol) || (size(sol,1)~=1) || (length(sol) > 1)) res = false;else res=true ;end']),'binfo ExitFlag Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol=binfo.Iterations;if(~isreal(sol) || (size(sol,1)~=1) || (length(sol) > 1)) res = false;else res=true ;end']),'binfo Iterations Test Failed');

assert(output_ctrl(['load("Matrix.mat");','[sol,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);if(~isreal(sol) || (size(sol,1)==1) || (size(sol,1) ~= n)) res = false;else res=true ;end']),'ainfo sol Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol=binfo.Setup_Time;if(~isreal(sol) || (size(sol,1)~=1) || (length(sol) > 1)) res = false;else res=true ;end']),'ainfo Setup_Time Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol=binfo.Solve_Time;if(~isreal(sol) || (size(sol,1)~=1) || (length(sol) > 1)) res = false;else res=true ;end']),'ainfo Solve_Time Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol=binfo.ExitFlag;if(~isreal(sol) || (size(sol,1)~=1) || (length(sol) > 1)) res = false;else res=true ;end']),'ainfo ExitFlag Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol=binfo.Iterations;if(~isreal(sol) || (size(sol,1)~=1) || (length(sol) > 1)) res = false;else res=true ;end']),'ainfo Iterations Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol=ainfo.KKT_Time;if(~isreal(sol) || (size(sol,1)~=1) || (length(sol) > 1)) res = false;else res=true ;end']),'ainfo KKT_Time Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol=ainfo.LDL_Time;if(~isreal(sol) || (size(sol,1)~=1) || (length(sol) > 1)) res = false;else res=true ;end']),'ainfo LDL_Time Test Failed');

assert(output_ctrl(['load("Matrix.mat");','[x,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol = ainfo.y;if(~isreal(sol) || (size(sol,1)==1) || (size(sol,1) ~= p)) res = false;else res=true ;end']),'ainfo y Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol = ainfo.z;if(~isreal(sol) || (size(sol,1)==1) || (size(sol,1) ~= m)) res = false;else res=true ;end']),'ainfo z Test Failed');
assert(output_ctrl(['load("Matrix.mat");','[x,binfo,ainfo]=qpSWIFT(sparse(P),c,sparse(A),b,sparse(G),h);sol = ainfo.s;if(~isreal(sol) || (size(sol,1)==1) || (size(sol,1) ~= m)) res = false;else res=true ;end']),'ainfo s Test Failed');

disp('Syntax Tests Passed !!!!');

function res = error_ctrl(cmd,msg)
    try
        eval(cmd);
        res = false;
    catch ME
        if(strcmp(ME.message,msg))
            res = true;
        else
            res = false;
        end
    end
end


function res = output_ctrl(cmd)

    eval(cmd);

end