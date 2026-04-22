function out = simulate_robot(p, sim)
%SIMULATE_ROBOT Run robot dynamics with controller and disturbance callbacks.

ode_fun = @(t,x) state_derivative_3r(t, x, p, sim.control_fun(t,x,p) + sim.disturbance_fun(t,x,p));
[t, X] = ode45(ode_fun, sim.tspan, sim.x0, sim.ode);

n = numel(t);
q = X(:,1:3);
dq = X(:,4:6);
ee = zeros(n,3);
tau_cmd = zeros(n,3);
tau_ext = zeros(n,3);

for i = 1:n
    xi = X(i,:)';
    tau_cmd(i,:) = sim.control_fun(t(i), xi, p)';
    tau_ext(i,:) = sim.disturbance_fun(t(i), xi, p)';
    ee(i,:) = fkine_3r(q(i,:)', p)';
end

out.t = t;
out.X = X;
out.q = q;
out.dq = dq;
out.ee = ee;
out.tau_cmd = tau_cmd;
out.tau_ext = tau_ext;
out.tau_total = tau_cmd + tau_ext;

end
