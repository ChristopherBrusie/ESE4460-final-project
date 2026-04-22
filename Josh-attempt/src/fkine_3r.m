function ee = fkine_3r(q, p)
%FKINE_3R End-effector [x; y; alpha] for 3R planar arm.

q1 = q(1); q2 = q(2); q3 = q(3);
L1 = p.L(1); L2 = p.L(2); L3 = p.L(3);

ee = [L1*cos(q1) + L2*cos(q1+q2) + L3*cos(q1+q2+q3);
      L1*sin(q1) + L2*sin(q1+q2) + L3*sin(q1+q2+q3);
      q1 + q2 + q3];

end
