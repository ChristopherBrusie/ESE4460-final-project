function sim = default_sim_options(p)
%DEFAULT_SIM_OPTIONS Common simulation options.

sim.tspan = [0 5];
sim.x0 = [p.q0; p.dq0];
sim.ode = odeset('RelTol',1e-7,'AbsTol',1e-9);
sim.control_fun = @(t,x,p_) zeros(3,1); %#ok<INUSD>
sim.disturbance_fun = @(t,x,p_) zeros(3,1); %#ok<INUSD>

end
