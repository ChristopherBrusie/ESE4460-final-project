function Jdot_dq = jacobian_dot_times_qdot(q, dq, p)
%JACOBIAN_DOT_TIMES_QDOT Numerical Jdot(q,dq)*dq.

h = 1e-6;
qf = q + h*dq;
qb = q - h*dq;
Jf = jacobian_3r(qf, p);
Jb = jacobian_3r(qb, p);
Jdot = (Jf - Jb)/(2*h);
Jdot_dq = Jdot*dq;

end
