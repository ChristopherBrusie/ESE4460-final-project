function [qd, dqd, ddqd] = eval_task3_trajectory(t, pp1, pp2, pp3)
%EVAL_TASK3_TRAJECTORY Evaluate qd, qd_dot, qd_ddot from spline pp forms.

h = 1e-4;
qd = [ppval(pp1,t); ppval(pp2,t); ppval(pp3,t)];
qp = [ppval(pp1,t+h); ppval(pp2,t+h); ppval(pp3,t+h)];
qm = [ppval(pp1,t-h); ppval(pp2,t-h); ppval(pp3,t-h)];

dqd = (qp - qm)/(2*h);
ddqd = (qp - 2*qd + qm)/(h^2);

end
