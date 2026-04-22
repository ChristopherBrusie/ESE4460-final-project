function run_task2()
%RUN_TASK2 Task-2: control partitioning — regulation, step response, and gain study.

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
p = robot_params();
plot_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'plots', 'task2');
if ~exist(plot_dir,'dir'), mkdir(plot_dir); end

step_deg = 10;
step_t   = 1.0;   % [s] time of step command

%% -----------------------------------------------------------------------
%% Part A: Initial-condition regulation — no step command
%  With the computed-torque controller holding q_d = q_0, the arm should
%  remain stationary (error = 0 for all time since initial state = desired).
%  This shows that the controller correctly compensates gravity + dynamics.
%% -----------------------------------------------------------------------
Kp_reg = diag([100 100 100]);
Kv_reg = diag([20  20  20]);
qd_hold = p.q0;

sim = default_sim_options(p);
sim.tspan = [0 5];
sim.control_fun = @(t,x,p_) computed_torque_pd( ...
    x(1:3), x(4:6), qd_hold, zeros(3,1), zeros(3,1), p_, Kp_reg, Kv_reg);

out_reg = simulate_robot(p, sim);

fig_reg = figure('Color','w','Name','Task2 PartA Regulation','Position',[80 80 1200 500]);
subplot(1,2,1);
plot(out_reg.t, rad2deg(out_reg.q), 'LineWidth',1.7);
grid on; xlabel('t [s]'); ylabel('q [deg]');
legend('q_1','q_2','q_3','Location','best');
title('Regulation at q_0 — joint angles constant');
subplot(1,2,2);
plot(out_reg.t, out_reg.tau_cmd, 'LineWidth',1.7);
grid on; xlabel('t [s]'); ylabel('tau [N m]');
legend('tau1','tau2','tau3','Location','best');
title('Control torques (steady gravity compensation)');
sgtitle('Task 2 Part A: Regulation (K_p=100, K_v=20)','FontWeight','bold');
save_fig(fig_reg, plot_dir, 'partA_regulation');

%% -----------------------------------------------------------------------
%% Part B: Step response — one joint at a time, Kp sweep [1, 10, 100]
%  Kv = 2*sqrt(Kp) gives critically-damped error dynamics for each Kp.
%% -----------------------------------------------------------------------
kp_vals = [1, 10, 100];
metrics = zeros(3, numel(kp_vals), 2);   % (joint, Kp_idx, [rise_t, overshoot])

for joint = 1:3
    fig_sw = figure('Color','w','Name',sprintf('Task2 PartB Joint%d',joint), ...
                    'Position',[80 80 1400 900]);

    for ki = 1:numel(kp_vals)
        kp = kp_vals(ki);
        Kp = diag(repmat(kp, 1, 3));
        Kv = diag(2*sqrt(repmat(kp, 1, 3)));   % critical damping

        qd0   = p.q0;
        qd1   = p.q0;  qd1(joint) = qd1(joint) + deg2rad(step_deg);

        sim = default_sim_options(p);
        sim.tspan = [0 6];
        sim.control_fun = @(t,x,p_) step_ctrl(t, x, p_, qd0, qd1, step_t, Kp, Kv);

        out = simulate_robot(p, sim);

        q_des = repmat(qd0', numel(out.t), 1);
        q_des(out.t >= step_t, joint) = qd1(joint);
        err = q_des - out.q;

        subplot(numel(kp_vals), 2, 2*ki-1);
        plot(out.t, rad2deg(out.q(:,joint)), 'LineWidth',1.7); hold on;
        plot(out.t, rad2deg(q_des(:,joint)), '--', 'LineWidth',1.7);
        grid on; xlabel('t [s]'); ylabel('q [deg]');
        legend('actual','desired','Location','best');
        title(sprintf('K_p=%d,  K_v=%.1f  (critical)', kp, 2*sqrt(kp)));

        subplot(numel(kp_vals), 2, 2*ki);
        plot(out.t, rad2deg(err(:,joint)), 'LineWidth',1.7);
        grid on; xlabel('t [s]'); ylabel('error [deg]'); title('Tracking Error');

        i0 = find(out.t >= step_t, 1);
        m = analyze_step_response(out.t(i0:end), out.q(i0:end,joint), ...
                                  out.q(i0,joint), qd1(joint));
        metrics(joint, ki, 1) = m.rise_time;
        metrics(joint, ki, 2) = m.overshoot_pct;
    end

    sgtitle(sprintf('Task 2 Part B: Joint-%d Step (+%d°) — K_p Sweep', joint, step_deg), ...
            'FontWeight','bold');
    save_fig(fig_sw, plot_dir, sprintf('partB_joint%d_kp_sweep', joint));
end

%% -----------------------------------------------------------------------
%% Part C: Overshoot demonstration — Kp=100, Kv=4 (heavily underdamped)
%  Critical Kv would be 2*sqrt(100)=20; using 4 shows clear oscillation.
%% -----------------------------------------------------------------------
Kp_od = diag([100 100 100]);
Kv_od = diag([4   4   4]);

for joint = 1:3
    qd1 = p.q0;  qd1(joint) = qd1(joint) + deg2rad(step_deg);

    sim = default_sim_options(p);
    sim.tspan = [0 6];
    sim.control_fun = @(t,x,p_) step_ctrl(t, x, p_, p.q0, qd1, step_t, Kp_od, Kv_od);
    out_od = simulate_robot(p, sim);

    q_des = repmat(p.q0', numel(out_od.t), 1);
    q_des(out_od.t >= step_t, joint) = qd1(joint);

    fig_od = figure('Color','w','Name',sprintf('Task2 PartC Overshoot J%d',joint), ...
                    'Position',[80 80 900 450]);
    subplot(1,2,1);
    plot(out_od.t, rad2deg(out_od.q(:,joint)), 'LineWidth',1.7); hold on;
    plot(out_od.t, rad2deg(q_des(:,joint)), '--', 'LineWidth',1.7);
    grid on; xlabel('t [s]'); ylabel('q [deg]');
    legend('actual','desired','Location','best');
    title(sprintf('Joint %d — K_p=100, K_v=4 (underdamped)', joint));
    subplot(1,2,2);
    plot(out_od.t, rad2deg(q_des(:,joint) - out_od.q(:,joint)), 'LineWidth',1.7);
    grid on; xlabel('t [s]'); ylabel('error [deg]'); title('Tracking Error');
    sgtitle('Task 2 Part C: Overshoot with Underdamped Gains','FontWeight','bold');
    save_fig(fig_od, plot_dir, sprintf('partC_overshoot_joint%d', joint));
end

%% -----------------------------------------------------------------------
%% Print answers to each assignment question
%% -----------------------------------------------------------------------
fprintf('\n=== Task 2 Analysis ===\n\n');

fprintf('Rise times [s]:\n');
fprintf('             Kp=%-4d  Kp=%-4d  Kp=%-4d\n', kp_vals);
for j = 1:3
    fprintf('  Joint %d    ', j);
    fprintf('%6.3f    ', metrics(j,:,1));
    fprintf('\n');
end

fprintf('\nQ1: How does rise time change with Kp?\n');
rt_ratio_1_10  = mean(metrics(:,1,1)) / mean(metrics(:,2,1));
rt_ratio_10_100 = mean(metrics(:,2,1)) / mean(metrics(:,3,1));
fprintf('    Rise time decreases as Kp increases (higher bandwidth).\n');
fprintf('    Kp=1→10: ~%.1fx faster.  Kp=10→100: ~%.1fx faster.\n', ...
        rt_ratio_1_10, rt_ratio_10_100);

fprintf('\nQ2: Can you set the gain to give overshoot?\n');
fprintf('    Yes — reduce Kv below the critically-damped value 2*sqrt(Kp).\n');
fprintf('    With Kp=100, Kv=4 (vs. critical Kv=20), clear oscillatory\n');
fprintf('    overshoot is observed in Part C figures.\n');

fprintf('\nQ3: Does the system need different Kp values per joint?\n');
spread = (max(metrics,[],1) - min(metrics,[],1)) ./ mean(metrics,1) * 100;
fprintf('    Rise-time spread across joints: %.1f%% (Kp=1), %.1f%% (Kp=10), %.1f%% (Kp=100).\n', ...
        spread(1,1,1), spread(1,2,1), spread(1,3,1));
fprintf('    Computed-torque cancels coupled nonlinearities, leaving decoupled\n');
fprintf('    2nd-order error dynamics per joint.  A single Kp works well.\n');

fprintf('\nQ4: Does the step response look like a second-order system?\n');
fprintf('    Yes.  The control law tau = M*(qdd_d + Kv*e_dot + Kp*e) + C*qdot + G\n');
fprintf('    exactly linearises the closed-loop to:  e_ddot + Kv*e_dot + Kp*e = 0.\n');
fprintf('    This is a second-order ODE, confirmed by the overdamped/underdamped\n');
fprintf('    shapes visible in the step-response plots.\n');

end

% -------------------------------------------------------------------------
function tau = step_ctrl(t, x, p, qd0, qd1, t_step, Kp, Kv)
q  = x(1:3);
dq = x(4:6);
if t < t_step
    qd = qd0;
else
    qd = qd1;
end
tau = computed_torque_pd(q, dq, qd, zeros(3,1), zeros(3,1), p, Kp, Kv);
end
