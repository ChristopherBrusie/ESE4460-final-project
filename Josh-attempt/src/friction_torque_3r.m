function tau_f = friction_torque_3r(dq, p)
%FRICTION_TORQUE_3R Viscous joint friction.

tau_f = -p.friction(:).*dq(:);

end
