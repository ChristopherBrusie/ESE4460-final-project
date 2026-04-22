function tau = computed_torque_pd(q, dq, qd, dqd, ddqd, p, Kp, Kv)
%COMPUTED_TORQUE_PD Model-based + servo partition controller.

M = mass_matrix_3r(q,p);
C = coriolis_matrix_3r(q,dq,p);
G = gravity_vector_3r(q,p);

e = qd - q;
ed = dqd - dq;
v = ddqd + Kv*ed + Kp*e;

tau = M*v + C*dq + G;

end
