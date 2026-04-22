function C = coriolis_matrix_3r(q, dq, p)
%CORIOLIS_MATRIX_3R Coriolis/centrifugal matrix via Christoffel symbols.

n = 3;
h = 1e-7;
dMdq = zeros(n,n,n);

for k = 1:n
    qp = q; qp(k) = q(k) + h;
    qm = q; qm(k) = q(k) - h;
    dMdq(:,:,k) = (mass_matrix_3r(qp,p) - mass_matrix_3r(qm,p)) / (2*h);
end

C = zeros(n,n);
for i = 1:n
    for j = 1:n
        cij = 0;
        for k = 1:n
            gamma = 0.5*(dMdq(i,j,k) + dMdq(i,k,j) - dMdq(j,k,i));
            cij = cij + gamma*dq(k);
        end
        C(i,j) = cij;
    end
end

end
