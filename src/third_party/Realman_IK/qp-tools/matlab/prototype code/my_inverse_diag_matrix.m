function invW = my_inverse_diag_matrix(W)
[n,m] = size(W);
invW = zeros(n,m);
for ii=1:n
    invW(ii,ii) = 1/W(ii,ii);
end