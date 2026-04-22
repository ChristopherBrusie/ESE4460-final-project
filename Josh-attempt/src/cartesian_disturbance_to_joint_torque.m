function tau_ext = cartesian_disturbance_to_joint_torque(q, Fxyz, p)
%CARTESIAN_DISTURBANCE_TO_JOINT_TORQUE Maps [Fx;Fy;Ma] to joint torques.

J = jacobian_3r(q, p);
tau_ext = J' * Fxyz(:);

end
