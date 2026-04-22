function G = gravity_vector_3r(q, p)
%GRAVITY_VECTOR_3R Joint gravity vector G(q).

q1=q(1); q2=q(2); q3=q(3);
c1=cos(q1); c12=cos(q1+q2); c123=cos(q1+q2+q3);
L1=p.L(1); L2=p.L(2);
lc1=p.lc(1); lc2=p.lc(2); lc3=p.lc(3);

G = zeros(3,1);
G(1) = p.g*(p.m(1)*lc1*c1 + p.m(2)*(L1*c1 + lc2*c12) + p.m(3)*(L1*c1 + L2*c12 + lc3*c123));
G(2) = p.g*(p.m(2)*lc2*c12 + p.m(3)*(L2*c12 + lc3*c123));
G(3) = p.g*(p.m(3)*lc3*c123);

end
