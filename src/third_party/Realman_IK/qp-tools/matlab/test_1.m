clc;
clearvars;
close all;

addpath(genpath(pwd));
warning('off','all');

%%% Number of Decision Variables
idx = 2;
cnt = 0;
cnt0 = 0;
cnt1 = 0;
cnt2 = 0;
dcnt0 = [];
dcnt1 = [];
dcnt2 = [];

%%% Number of equality Constraints
eq_cons = 1;

H = waitbar(0,'Please Wait...');

samples= 1e3;
for  i = 1:samples
    
    PP = randn(idx,idx);
    
    P = PP'*PP + 10*eye(idx);
    
    q = randn(idx,1);
    
    A = randn(eq_cons,idx);
    
    b = randn(eq_cons,1);
    
    G = -eye(idx);
    
    h = zeros(idx,1);
    
    options = optimoptions('quadprog','Display','off');
    [X,FVAL,EXITFLAG] = quadprog(P,q,G,h,A,b,[],[],[],options);
    [sol1,basic_info1,adv_info1] = qpSWIFT(sparse(P),q,sparse(A),b,sparse(G),h);
%     [sol1,basic_info1,adv_info1] = qpSWIFT(sparse(P),q,sparse(G),h,opts);
    %     [sol2,basic_info2,adv_info2] = qpSWIFT_pd_united(sparse(P),q,sparse(A),b,sparse(G),h,opts);
%     [x,s,fval,jj,x_sol,s_sol,z_sol] = qpSWIFT_matlab(P,q,G,h,A,b);
[x,s,fval,jj,x_sol,s_sol,z_sol] = customQP_PredCorr_full(P,q,G,h,A,b);
    
    
    if  EXITFLAG == 1
        cnt = cnt + 1;
        
        if (basic_info1.ExitFlag == 0)
            cnt0 = cnt0 + 1;
        else
            dcnt0 = [dcnt0;i];
        end
        
%         if (basic_info2.ExitFlag == 0)
%             cnt1 = cnt1 + 1;
%         else
%             dcnt1 = [dcnt1;i];
%             nd = length(dcnt1);
%             MM(nd).P = P;
%             MM(nd).A = A;
%             MM(nd).q = q;
%             MM(nd).b = b;
%             MM(nd).G = G;
%             MM(nd).h = h;
%         end
        
        if (jj < 100)
            cnt2 = cnt2 + 1;
        else
            dcnt2 = [dcnt2;i];
        end
        
    end
    
    waitbar(i/samples,H);
    
end
close(H);

% M = categorical({['QUADPROG ',num2str(cnt)],['qpSWIFT ',num2str(cnt0)],['MqpSWIFT ',num2str(cnt1)],['CustomQP ',num2str(cnt2)]});
M = categorical({['QUADPROG ',num2str(cnt)],['qpSWIFT ',num2str(cnt0)],['CustomQP ',num2str(cnt2)]});


hB = bar(M,[cnt,cnt0,cnt2]);
title(['Solved Instances out of ',num2str(samples),' samples']);