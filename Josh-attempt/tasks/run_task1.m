function run_task1()
%RUN_TASK1 Task-1: five simulation cases demonstrating 3R planar robot dynamics.

addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src'));
p_base = robot_params();
plot_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'plots', 'task1');
if ~exist(plot_dir,'dir'), mkdir(plot_dir); end

q0 = p_base.q0;
G0 = gravity_vector_3r(q0, p_base);   % static gravity torques at q0

fprintf('Task 1 — Gravity torques at q0=[10,20,30] deg:\n');
fprintf('  tau_g = [%.3f,  %.3f,  %.3f] N·m\n\n', G0(1), G0(2), G0(3));

% Each case: [gravity flag, applied torque vector]
cases = {
    'case1_no_tau_no_gravity',   0,        [0;0;0];
    'case2_no_tau_with_gravity', p_base.g, [0;0;0];
    'case3_tau1_equilibrium',    p_base.g, [G0(1);0;0];
    'case4_tau12_equilibrium',   p_base.g, [G0(1);G0(2);0];
    'case5_tau123_equilibrium',  p_base.g, G0
};
labels = {
    'No torque, no gravity';
    'No torque, with gravity';
    'tau1 equilibrium, gravity';
    'tau12 equilibrium, gravity';
    'Full equilibrium, gravity'
};

for k = 1:size(cases,1)
    case_name = cases{k,1};
    g_val     = cases{k,2};
    tau_app   = cases{k,3};

    p = p_base;
    p.g = g_val;

    sim = default_sim_options(p);
    sim.tspan = [0 5];
    sim.control_fun = @(t,x,p_) tau_app; %#ok<INUSD>

    out = simulate_robot(p, sim);

    fig = figure('Color','w','Name', labels{k}, 'Position',[50 50 1400 750]);

    subplot(2,3,1);
    plot(out.t, rad2deg(out.q), 'LineWidth',1.6);
    grid on; xlabel('t [s]'); ylabel('q [deg]');
    legend('q_1','q_2','q_3','Location','best'); title('Joint Angles');

    subplot(2,3,2);
    plot(out.t, out.dq, 'LineWidth',1.6);
    grid on; xlabel('t [s]'); ylabel('qdot [rad/s]');
    legend('qdot1','qdot2','qdot3','Location','best'); title('Joint Velocities');

    subplot(2,3,3);
    plot(out.t, out.ee(:,1), 'LineWidth',1.6); hold on;
    plot(out.t, out.ee(:,2), 'LineWidth',1.6);
    plot(out.t, rad2deg(out.ee(:,3)), 'LineWidth',1.6);
    grid on; xlabel('t [s]');
    legend('x [m]','y [m]','alpha [deg]','Location','best');
    title('End-Effector Pose');

    subplot(2,3,4);
    % Arm snapshots: t=0 (blue) and t=T (red)
    for si = [1, numel(out.t)]
        pos = arm_keypoints(out.q(si,:)', p);
        c = 'b'; if si > 1, c = 'r'; end
        plot(pos(:,1), pos(:,2), '-o', 'Color',c, 'LineWidth',2, 'MarkerFaceColor',c);
        hold on;
    end
    grid on; axis equal;
    lim = sum(p.L) + 0.5;
    axis([-lim lim -lim lim]);
    xlabel('x [m]'); ylabel('y [m]');
    legend('t = 0','t = T','Location','best'); title('Arm Snapshot');

    subplot(2,3,5);
    tau_mat = repmat(tau_app', numel(out.t), 1);
    plot(out.t, tau_mat, 'LineWidth',1.6);
    grid on; xlabel('t [s]'); ylabel('[N·m]');
    legend('tau1','tau2','tau3','Location','best'); title('Applied Torques');

    subplot(2,3,6);
    plot(out.ee(:,1), out.ee(:,2), 'k-', 'LineWidth',1.5); hold on;
    plot(out.ee(1,1),   out.ee(1,2),   'go','MarkerSize',8,'MarkerFaceColor','g');
    plot(out.ee(end,1), out.ee(end,2), 'rs','MarkerSize',8,'MarkerFaceColor','r');
    grid on; axis equal; xlabel('x [m]'); ylabel('y [m]');
    legend('path','start','end','Location','best'); title('EE Trajectory');

    sgtitle(sprintf('Task 1 — Case %d: %s', k, labels{k}), 'FontWeight','bold');
    save_fig(fig, plot_dir, case_name);

    fprintf('Case %d  %-35s  EE_final = (%.3f m, %.3f m, %.1f deg)\n', ...
        k, labels{k}, out.ee(end,1), out.ee(end,2), rad2deg(out.ee(end,3)));
end

end

% -------------------------------------------------------------------------
function pos = arm_keypoints(q, p)
% Returns [x,y] for base, joint 1, joint 2, end-effector.
q1 = q(1); q2 = q(2); q3 = q(3);
x1 = p.L(1)*cos(q1);                           y1 = p.L(1)*sin(q1);
x2 = x1 + p.L(2)*cos(q1+q2);                  y2 = y1 + p.L(2)*sin(q1+q2);
x3 = x2 + p.L(3)*cos(q1+q2+q3);               y3 = y2 + p.L(3)*sin(q1+q2+q3);
pos = [0 0; x1 y1; x2 y2; x3 y3];
end
