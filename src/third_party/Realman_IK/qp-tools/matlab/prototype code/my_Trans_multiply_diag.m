function WTW = my_Trans_multiply_diag(W)
[n,m] = size(W);
WTW = zeros(n,m);
for ii=1:n
    WTW(ii,ii) = W(ii,ii)^2;
end