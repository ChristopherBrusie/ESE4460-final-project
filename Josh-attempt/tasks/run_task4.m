function run_task4()
%RUN_TASK4 Task-4: end-effector force/torque disturbances mapped through Jacobian.
%
%  Scenario A — default config q0=[10,20,30] deg.  Three sequential impulses:
%               Fx at t=1.5s,  Fy at t=3.0s,  moment Ma at t=4.5s.
%
%  Scenario B — vertical arm q=[90,0,0] deg.  Downward Fy impulse.
%               The Jacobian at this configuration concentrates the force
%               primarily into joint-1, demonstrating position-dependent
%               force mapping (interesting singularity-adjacent behaviour).
%
%  Scenario C — default config, large alpha (end-effector torque) impulse.
%               Shows that Ma distributes equally across all three joints
%               (third row of J' = [1,1,1] regardless of configuration).

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
p = robot_params();
plot_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'plots', 'task4');
if ~exist(plot_dir,'dir'), mkdir(plot_dir); end

Kp = diag([80 80 80]);
Kv = diag([18 18 18]);

run_scenario(p, Kp, Kv, p.q0,              @force_ABC,    'scenA', plot_dir, ...
    'Scenario A: Default Config - Fx (t=1.5s), Fy (t=3.0s), Ma (t=4.5s)');

run_scenario(p, Kp, Kv, deg2rad([90;0;0]), @force_downY,  'scenB', plot_dir, ...
    'Scenario B: Vertical Arm — Downward F_y Impulse (t=2.0s)');

run_scenario(p, Kp, Kv, p.q0,              @force_largeMa,'scenC', plot_dir, ...
    'Scenario C: Default Config - Large Ma Impulse (t=2.5s)');

end

% =========================================================================
function run_scenario(p, Kp, Kv, qd, force_fun, filestem, plot_dir, ttl)

p_sc       = p;
p_sc.q0    = qd;
p_sc.dq0   = zeros(3,1);

sim = default_sim_options(p_sc);
sim.tspan  = [0 6];
sim.x0     = [qd; zeros(3,1)];
sim.control_fun    = @(t,x,p_) computed_torque_pd( ...
    x(1:3), x(4:6), qd, zeros(3,1), zeros(3,1), p_, Kp, Kv);
sim.disturbance_fun = @(t,x,p_) J_transpose_map(t, x, p_, force_fun);

out = simulate_robot(p_sc, sim);

% Reconstruct Cartesian force history
Fhist = zeros(numel(out.t), 3);
for i = 1:numel(out.t)
    Fhist(i,:) = force_fun(out.t(i))';
end

% Print Jacobian columns at the nominal config so we can explain coupling
J_nom = jacobian_3r(qd, p);
fprintf('\n%s\n', ttl);
fprintf('  J^T at nominal config:\n');
fprintf('    [%.3f  %.3f  %.3f]\n', J_nom(:,1));
fprintf('    [%.3f  %.3f  %.3f]\n', J_nom(:,2));
fprintf('    [%.3f  %.3f  %.3f]\n', J_nom(:,3));

fig = figure('Color','w','Name', ttl, 'Position',[80 80 1400 800]);

subplot(2,2,1);
plot(out.t, Fhist, 'LineWidth',1.6);
grid on; xlabel('t [s]');
legend('Fx [N]','Fy [N]','Ma [N m]','Location','best');
title('Applied Cartesian Disturbance');

subplot(2,2,2);
plot(out.t, rad2deg(out.q), 'LineWidth',1.6);
grid on; xlabel('t [s]'); ylabel('q [deg]');
legend('q_1','q_2','q_3','Location','best'); title('Joint Response');

subplot(2,2,3);
plot(out.t, out.ee(:,1:2), 'LineWidth',1.6);
grid on; xlabel('t [s]'); ylabel('[m]');
legend('x','y','Location','best'); title('End-Effector Position');

subplot(2,2,4);
plot(out.t, out.tau_ext, 'LineWidth',1.6);
grid on; xlabel('t [s]'); ylabel('[N·m]');
legend('tau_ext1','tau_ext2','tau_ext3','Location','best');
title('Equivalent Joint Disturbance Torques  (J^T \cdot F)');

sgtitle(ttl, 'FontWeight','bold');
save_fig(fig, plot_dir, filestem);
end

% =========================================================================
function tau_ext = J_transpose_map(t, x, p, force_fun)
tau_ext = cartesian_disturbance_to_joint_torque(x(1:3), force_fun(t), p);
end

% ---- Force profiles ------------------------------------------------------
function F = force_ABC(t)
F = [0;0;0];
if     t >= 1.50 && t <= 1.55,  F = [60;   0;   0];   % Fx impulse
elseif t >= 3.00 && t <= 3.05,  F = [ 0; -80;   0];   % Fy impulse (downward)
elseif t >= 4.50 && t <= 4.55,  F = [ 0;   0;  30];   % Ma impulse
end
end

function F = force_downY(t)
% Downward force on a nearly-singular vertical configuration.
% J^T at q=[90,0,0]: the Fy column is [L1+L2+L3, L2+L3, L3]' = [9,5,2]'
% so joint 1 absorbs most of the load — controller must respond strongly.
F = [0;0;0];
if t >= 2.00 && t <= 2.05,  F = [0; -100; 0]; end
end

function F = force_largeMa(t)
% End-effector moment.  J^T row 3 = [1,1,1], so Ma is distributed equally
% to all three joints regardless of configuration.
F = [0;0;0];
if t >= 2.50 && t <= 2.55,  F = [0; 0; 60]; end
end
