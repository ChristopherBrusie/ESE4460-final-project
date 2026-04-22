function fig = plot_joint_and_ee(out, title_text)
%PLOT_JOINT_AND_EE Plot q, dq, x/y/alpha, and torques.

fig = figure('Color','w','Name',title_text,'Position',[50 50 1400 800]);

subplot(2,2,1);
plot(out.t, rad2deg(out.q), 'LineWidth',1.6);
grid on; xlabel('Time [s]'); ylabel('q [deg]');
legend('q1','q2','q3','Location','best'); title('Joint Angles');

subplot(2,2,2);
plot(out.t, out.dq, 'LineWidth',1.6);
grid on; xlabel('Time [s]'); ylabel('dq [rad/s]');
legend('dq1','dq2','dq3','Location','best'); title('Joint Velocities');

subplot(2,2,3);
plot(out.t, out.ee(:,1), 'LineWidth',1.6); hold on;
plot(out.t, out.ee(:,2), 'LineWidth',1.6);
plot(out.t, rad2deg(out.ee(:,3)), 'LineWidth',1.6);
grid on; xlabel('Time [s]'); ylabel('EE');
legend('x [m]','y [m]','alpha [deg]','Location','best');
title('End-Effector Pose');

subplot(2,2,4);
plot(out.t, out.tau_total, 'LineWidth',1.4);
grid on; xlabel('Time [s]'); ylabel('tau [N m]');
legend('tau1','tau2','tau3','Location','best'); title('Total Joint Torque');

sgtitle(title_text, 'FontWeight','bold');

end
