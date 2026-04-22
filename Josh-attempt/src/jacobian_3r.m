function J = jacobian_3r(q, p)
%JACOBIAN_3R Planar Jacobian mapping qdot -> [xdot; ydot; alphadot].

q1 = q(1); q2 = q(2); q3 = q(3);
L1 = p.L(1); L2 = p.L(2); L3 = p.L(3);

J = [ -L1*sin(q1) - L2*sin(q1+q2) - L3*sin(q1+q2+q3), -L2*sin(q1+q2) - L3*sin(q1+q2+q3), -L3*sin(q1+q2+q3);
       L1*cos(q1) + L2*cos(q1+q2) + L3*cos(q1+q2+q3),  L2*cos(q1+q2) + L3*cos(q1+q2+q3),  L3*cos(q1+q2+q3);
       1,                                                1,                                   1];

end
