function M = mass_matrix_3r(q, p)
%MASS_MATRIX_3R Joint-space inertia matrix M(q).

q1=q(1); q2=q(2); q3=q(3);
s1=sin(q1); c1=cos(q1);
s12=sin(q1+q2); c12=cos(q1+q2);
s123=sin(q1+q2+q3); c123=cos(q1+q2+q3);

L1=p.L(1); L2=p.L(2);
lc1=p.lc(1); lc2=p.lc(2); lc3=p.lc(3);

Jv1 = [-lc1*s1, 0, 0;
        lc1*c1, 0, 0];

Jv2 = [-L1*s1 - lc2*s12, -lc2*s12, 0;
        L1*c1 + lc2*c12,  lc2*c12, 0];

Jv3 = [-L1*s1 - L2*s12 - lc3*s123, -L2*s12 - lc3*s123, -lc3*s123;
        L1*c1 + L2*c12 + lc3*c123,  L2*c12 + lc3*c123,   lc3*c123];

Jw1 = [1, 0, 0];
Jw2 = [1, 1, 0];
Jw3 = [1, 1, 1];

M = p.m(1)*(Jv1'*Jv1) + p.Iz(1)*(Jw1'*Jw1) + ...
    p.m(2)*(Jv2'*Jv2) + p.Iz(2)*(Jw2'*Jw2) + ...
    p.m(3)*(Jv3'*Jv3) + p.Iz(3)*(Jw3'*Jw3);

end
