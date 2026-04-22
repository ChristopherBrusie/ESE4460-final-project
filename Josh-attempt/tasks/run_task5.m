function run_task5()
%RUN_TASK5 Task-5: Cartesian controller — x, y, and alpha demonstrations.
%
%  The controller uses: tau = M*J^+(xdd_d + Dx*e_dot + Kx*e - Jdot*qdot) + C*qdot + G
%  which achieves second-order Cartesian error dynamics: e_ddot + Dx*e_dot + Kx*e = 0.
%
%  We first search the joint space for the configuration with maximum manipulability
%  w = |det(J)|, then demonstrate small independent motions in x, y, and alpha from
%  that configuration.  High manipulability avoids near-singular Jacobians that would
%  amplify errors through the pseudoinverse.

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
p = robot_params();
plot_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'plots', 'task5');
if ~exist(plot_dir,'dir'), mkdir(plot_dir); end

%% Find highest-manipulability configuration via grid search
q_best = find_max_manipulability_config(p);
w_best = manip(q_best, p);
x0     = fkine_3r(q_best, p);

fprintf('Task 5 — best configuration:\n');
fprintf('  q = [%.1f,  %.1f,  %.1f] deg\n', rad2deg(q_best'));
fprintf('  manipulability w = %.4f\n', w_best);
fprintf('  EE at x=%.3f m,  y=%.3f m,  alpha=%.1f deg\n\n', ...
        x0(1), x0(2), rad2deg(x0(3)));

%% Cartesian PD gains  (error dynamics: e_ddot + Dx*e_dot + Kx*e = 0)
Kx = diag([20 20 15]);
Dx = diag([9   9  8]);

p_mod      = p;
p_mod.q0   = q_best;
p_mod.dq0  = zeros(3,1);

sim = default_sim_options(p_mod);
sim.tspan = [0 12];
sim.x0    = [q_best; zeros(3,1)];
sim.control_fun = @(t,x,p_) cart_ctrl(t, x, p_, x0, Kx, Dx);

out = simulate_robot(p_mod, sim);

xd_hist = zeros(numel(out.t), 3);
for i = 1:numel(out.t)
    xd_hist(i,:) = cart_ref(out.t(i), x0)';
end

%% Plotting
ax_labels = {'x [m]', 'y [m]', 'alpha [deg]'};

fig = figure('Color','w','Name','Task5 Cartesian Control','Position',[80 80 1400 900]);

for ax = 1:3
    subplot(3,3,ax);
    if ax < 3
        actual  = out.ee(:,ax);
        desired = xd_hist(:,ax);
        unit = '[m]';
    else
        actual  = rad2deg(out.ee(:,ax));
        desired = rad2deg(xd_hist(:,ax));
        unit = '[deg]';
    end
    plot(out.t, actual,  'LineWidth',1.7); hold on;
    plot(out.t, desired, '--', 'LineWidth',1.7);
    grid on; xlabel('t [s]'); ylabel(unit);
    legend('actual','desired','Location','best');
    title(ax_labels{ax});
end

subplot(3,3,4:5);
ee_err = out.ee - xd_hist;
plot(out.t, [ee_err(:,1:2), rad2deg(ee_err(:,3))], 'LineWidth',1.6);
grid on; xlabel('t [s]'); ylabel('error');
legend('ex [m]','ey [m]','ealpha [deg]','Location','best');
title('Cartesian Tracking Error');

subplot(3,3,6);
plot(out.ee(:,1), out.ee(:,2), 'LineWidth',2); hold on;
plot(xd_hist(:,1), xd_hist(:,2), '--', 'LineWidth',2);
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]');
legend('actual','desired','Location','best'); title('xy Path');

subplot(3,3,7:8);
plot(out.t, out.tau_cmd, 'LineWidth',1.4);
grid on; xlabel('t [s]'); ylabel('tau [N m]');
legend('tau1','tau2','tau3','Location','best'); title('Control Torques');

subplot(3,3,9);
plot(out.t, rad2deg(out.q), 'LineWidth',1.4);
grid on; xlabel('t [s]'); ylabel('q [deg]');
legend('q_1','q_2','q_3','Location','best'); title('Joint Angles');

sgtitle('Task 5: Cartesian Controller - Independent x, y, alpha Motion', ...
        'FontWeight','bold');
save_fig(fig, plot_dir, 'task5_cartesian_control');

rms_e = sqrt(mean(ee_err.^2));
fprintf('Tracking RMS:  x=%.4f m,  y=%.4f m,  alpha=%.4f deg\n', ...
        rms_e(1), rms_e(2), rad2deg(rms_e(3)));

end

% =========================================================================
function q_best = find_max_manipulability_config(p)
% Coarse grid search over joint space; returns q with largest |det(J)|.
q1v = deg2rad(20:30:160);
q2v = deg2rad(-60:20:60);
q3v = deg2rad(-60:20:60);
w_max  = 0;
q_best = p.q0;
for a = q1v
    for b = q2v
        for c = q3v
            w = manip([a;b;c], p);
            if w > w_max
                w_max  = w;
                q_best = [a;b;c];
            end
        end
    end
end
end

function w = manip(q, p)
J = jacobian_3r(q, p);
w = abs(det(J));   % manipulability for a square (3×3) Jacobian
end

% =========================================================================
function tau = cart_ctrl(t, x, p, x0, Kx, Dx)
q  = x(1:3);
dq = x(4:6);

xd       = cart_ref(t, x0);
x_now    = fkine_3r(q, p);
J        = jacobian_3r(q, p);
xdot_now = J * dq;
Jdot_dq  = jacobian_dot_times_qdot(q, dq, p);

e  = xd - x_now;
ed = -xdot_now;   % desired xdot = 0 for piecewise-constant reference

qdd_cmd = pinv(J) * (Dx*ed + Kx*e - Jdot_dq);

M = mass_matrix_3r(q, p);
C = coriolis_matrix_3r(q, dq, p);
G = gravity_vector_3r(q, p);
tau = M*qdd_cmd + C*dq + G;
end

function xd = cart_ref(t, x0)
% Sequential step commands: x (t=2–5s), y (t=6–9s), alpha (t=10s+).
% Small steps (0.15 m / 10 deg) chosen to stay well inside the work envelope.
delta_xy    = 0.15;          % [m]
delta_alpha = deg2rad(10);   % [rad]

xd = x0;
if     t >= 2.0 && t < 5.0,  xd(1) = x0(1) + delta_xy;
elseif t >= 6.0 && t < 9.0,  xd(2) = x0(2) + delta_xy;
elseif t >= 10.0,             xd(3) = x0(3) + delta_alpha;
end
end
