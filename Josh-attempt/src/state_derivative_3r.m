function dx = state_derivative_3r(~, x, p, tau_total)
%STATE_DERIVATIVE_3R xdot = [dq; ddq] for robot dynamics.

q = x(1:3);
dq = x(4:6);

M = mass_matrix_3r(q, p);
C = coriolis_matrix_3r(q, dq, p);
G = gravity_vector_3r(q, p);
tau_f = friction_torque_3r(dq, p);

ddq = M \ (tau_total - C*dq - G + tau_f);
dx = [dq; ddq];

end
