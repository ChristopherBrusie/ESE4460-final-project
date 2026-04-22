
%% ================================================================
clear; clc; close all;

%% ── Robot Parameters ──────────────────────────────────────────
p.L  = [4;  3;  2 ];       % Link lengths           [m]
p.m  = [20; 15; 10];       % Link masses            [kg]
p.Iz = [0.5; 0.2; 0.1];    % Z-axis (planar) inertia [kg·m²]
p.lc = p.L / 2;            % CoM at link midpoint
p.g  = 9.81;               % Gravitational accel.   [m/s²]
p.friction = [20; 15; 10]; % viscous friction constants

%% ── Initial Conditions ─────────────────────────────────────────
q0  = [10; 20; 30] * (pi/180);   % Initial angles    [rad]
dq0 = [0;   0;  0];              % Initial velocities [rad/s]
x0  = [q0; dq0];                 % Full 6×1 state

%% ── Print Initial M, G for Verification ───────────────────────
fprintf('\n=== Initial Configuration ===\n');
fprintf('q0 = [%.1f  %.1f  %.1f] deg\n', q0*180/pi);
M0 = mass_matrix(q0, p);
G0 = gravity_vector(q0, p);
fprintf('\nMass Matrix M(q0):\n');
disp(M0);
fprintf('Gravity Vector G(q0) [N·m]:\n');
disp(G0);
fprintf('Static hold torques = G(q0):\n  tau = [%.3f  %.3f  %.3f] N·m\n\n', G0);

%% ── ODE45 Integration ─────────────────────────────────────────
fprintf('Simulating dynamics (0 → 5 s, τ = 0)...\n');
opts   = odeset('RelTol',1e-7,'AbsTol',1e-9);
tau    = zeros(3,1);   % No applied torques — free motion under gravity
[t, X] = ode45(@(t,x) state_space(t, x, tau, p), [0 5], x0, opts);
fprintf('Done  –  %d time steps computed.\n\n', length(t));

q  = X(:, 1:3);   % Joint angles      [rad]
dq = X(:, 4:6);   % Joint velocities  [rad/s]

%% ── Post-Process: Energy & Dynamic Terms ──────────────────────
n   = length(t);
KE  = zeros(n,1);   PE  = zeros(n,1);
Gnorm = zeros(n,1);

for i = 1:n
    Mi       = mass_matrix(q(i,:)', p);
    KE(i)    = 0.5 * dq(i,:) * Mi * dq(i,:)';
    PE(i)    = potential_energy(q(i,:)', p);
    Gnorm(i) = norm(gravity_vector(q(i,:)', p));
end
E_total = KE + PE;

%% ── Figure 1: Joint Angles ────────────────────────────────────
figure('Name','Joint Angles','Color','w','Position',[50 550 950 380]);
clrs = {[0.20 0.45 0.78],[0.85 0.33 0.10],[0.17 0.63 0.17]};
for j = 1:3
    subplot(1,3,j);
    plot(t, q(:,j)*180/pi, 'Color',clrs{j}, 'LineWidth',2);
    xlabel('Time [s]');  ylabel('Angle [deg]');
    title(sprintf('q_%d(t)', j));
    grid on;  box on;
end
sgtitle('Joint Angles — Free Motion Under Gravity', ...
        'FontSize',13,'FontWeight','bold');

%% ── Figure 2: Joint Velocities ────────────────────────────────
figure('Name','Joint Velocities','Color','w','Position',[50 130 950 380]);
for j = 1:3
    subplot(1,3,j);
    plot(t, dq(:,j), 'Color',clrs{j}, 'LineWidth',2);
    xlabel('Time [s]');  ylabel('Velocity [rad/s]');
    title(sprintf('\\dot{q}_%d(t)', j));
    grid on;  box on;
end
sgtitle('Joint Velocities — Free Motion Under Gravity', ...
        'FontSize',13,'FontWeight','bold');

%% ── Figure 3: Energy Conservation ────────────────────────────
figure('Name','Energy Conservation','Color','w','Position',[1020 550 580 360]);
plot(t, KE,      'b-',  'LineWidth',1.8, 'DisplayName','Kinetic  KE');  hold on;
plot(t, PE,      'r-',  'LineWidth',1.8, 'DisplayName','Potential PE');
plot(t, E_total, 'k--', 'LineWidth',2.0, 'DisplayName','Total  KE+PE');
xlabel('Time [s]');  ylabel('Energy [J]');
title('Energy Conservation (τ = 0)', 'FontSize',12,'FontWeight','bold');
legend('Location','best');  grid on;  box on;

fprintf('Energy drift (max - min) / mean = %.4f%%\n', ...
        100*(max(E_total)-min(E_total))/mean(abs(E_total)));

%% ── Figure 4: Gravity Vector Norm ─────────────────────────────
figure('Name','Gravity Vector Norm','Color','w','Position',[1020 130 580 360]);
plot(t, Gnorm, 'Color',[0.49 0.18 0.56], 'LineWidth',2);
xlabel('Time [s]');  ylabel('||G(q)|| [N·m]');
title('Gravity Vector Norm vs. Time','FontSize',12,'FontWeight','bold');
grid on;  box on;

%% ── Figure 5: Animation ───────────────────────────────────────
fig = figure('Name','3R Robot — Animation','Color','w', ...
             'Position',[50 50 620 590]);
ax  = axes('Parent',fig,'Color',[0.97 0.97 0.97]);
lim = sum(p.L) + 0.6;
axis(ax,[-lim lim -lim lim]);  axis(ax,'equal');
grid(ax,'on');  hold(ax,'on');
xlabel(ax,'X [m]');  ylabel(ax,'Y [m]');

% Ground line
plot(ax, [-lim lim], [0 0], 'k-', 'LineWidth',0.5, 'Color',[0.7 0.7 0.7]);

% Trajectory trail
h_trail = plot(ax, NaN, NaN, '--', 'LineWidth',1.0, ...
               'Color',[0.8 0.2 0.2], 'DisplayName','EE Trail');

% Link graphics  (each link separate for color)
link_col = {[0.20 0.45 0.78],[0.85 0.33 0.10],[0.17 0.63 0.17]};
h_links  = gobjects(1,3);
for j = 1:3
    h_links(j) = plot(ax, NaN, NaN, '-o', 'LineWidth',3.5, ...
        'Color',link_col{j}, 'MarkerFaceColor','k', ...
        'MarkerEdgeColor','k', 'MarkerSize',9);
end

% End-effector marker
h_ee = plot(ax, NaN, NaN, 'o', 'MarkerSize',12, ...
            'MarkerFaceColor','k', 'MarkerEdgeColor', 'k', 'LineWidth',1.5);

h_ttl = title(ax, 't = 0.00 s  —  3R Planar Robot (τ = 0)', ...
              'FontSize',11,'FontWeight','bold');

% Legend
legend(ax, [h_links(1), h_links(2), h_links(3), h_trail, h_ee], ...
       'Link 1','Link 2','Link 3','EE Trail','End-Effector', ...
       'Location','northeast','FontSize',8);

trail_x = [];  trail_y = [];
step = max(1, floor(n/300));   % ≈ 300 animation frames

for i = 1:step:n
    if ~ishandle(fig), break; end

    jts = forward_kinematics(q(i,1), q(i,2), q(i,3), p);  % 2×4

    % Update end-effector trail
    trail_x(end+1) = jts(1,4);                            
    trail_y(end+1) = jts(2,4);                             
    set(h_trail, 'XData',trail_x, 'YData',trail_y);

    % Update each link segment (joint i → joint i+1)
    for j = 1:3
        set(h_links(j), 'XData',jts(1,j:j+1), 'YData',jts(2,j:j+1));
    end
    set(h_ee, 'XData',jts(1,4), 'YData',jts(2,4));
    set(h_ttl,'String', sprintf('t = %.2f s  —  3R Planar Robot (\\tau = 0)', t(i)));
    drawnow;
end

fprintf('Final joint angles  :  q = [%.2f  %.2f  %.2f] deg\n', ...
        q(end,:)*180/pi);
fprintf('Final velocities    : dq = [%.3f  %.3f  %.3f] rad/s\n\n', ...
        dq(end,:));

%% ================================================================
%%  LOCAL FUNCTIONS
%% ================================================================

function dx = state_space(~, x, tau, p)
%STATE_SPACE  Computes ẋ for the Lagrangian EOM.
%
%   State:  x  = [q; q']     (6×1)
%   Output: dx = [q'; M\(τ - C*q' - G)]
%
    q  = x(1:3);
    dq = x(4:6);

    M   = mass_matrix(q, p);
    C   = coriolis_matrix(q, dq, p);
    G   = gravity_vector(q, p);
    tau_friction = friction(dq, p);


    % Solve M*q'' = tau - C*q' - G + tau_friction
    ddq = M \ (tau - C*dq - G + tau_friction);
    dx  = [dq; ddq];
end



%_________________________________________________________
function tau_friction = friction(dq, p)
% friction computes a torque due to viscous friction. 
% tau = b * q'
% 3x1 torque vector for each joint's friction, already negated. 
    tau_friction = -1*p.friction.*dq;
end


% ─────────────────────────────────────────────────────────────────
function M = mass_matrix(q, p)
%MASS_MATRIX  Computes the 3×3 joint-space inertia matrix M(q).
%
%   Derived via velocity Jacobians:
%       M = Σᵢ [ mᵢ·Jvᵢ'·Jvᵢ  +  Izᵢ·Jwᵢ'·Jwᵢ ]
%
%   Jvᵢ (2×3): linear  velocity Jacobian for CoM of link i
%   Jwᵢ (1×3): angular velocity Jacobian for link i
%              Jwᵢ(j) = 1  if j ≤ i,  else 0  (planar chain)
%
    q1=q(1);  q2=q(2);  q3=q(3);
    s1=sin(q1);       c1=cos(q1);
    s12=sin(q1+q2);   c12=cos(q1+q2);
    s123=sin(q1+q2+q3); c123=cos(q1+q2+q3);

    L1=p.L(1); L2=p.L(2);
    lc1=p.lc(1); lc2=p.lc(2); lc3=p.lc(3);

    % ── Linear velocity Jacobians ──────────────────────────────
    %   Row 1: d(x_ci)/d(qj)     Row 2: d(y_ci)/d(qj)
    Jv1 = [ -lc1*s1,                               0,              0  ;
              lc1*c1,                               0,              0  ];

    Jv2 = [ -L1*s1 - lc2*s12,            -lc2*s12,              0  ;
              L1*c1 + lc2*c12,             lc2*c12,              0  ];

    Jv3 = [ -L1*s1 - L2*s12 - lc3*s123,  -L2*s12 - lc3*s123,  -lc3*s123 ;
              L1*c1 + L2*c12 + lc3*c123,   L2*c12 + lc3*c123,   lc3*c123 ];

    % ── Angular velocity Jacobians (scalar ω per link) ─────────
    Jw1 = [1, 0, 0];
    Jw2 = [1, 1, 0];
    Jw3 = [1, 1, 1];

    % ── Assemble M ─────────────────────────────────────────────
    M = p.m(1)*(Jv1'*Jv1) + p.Iz(1)*(Jw1'*Jw1) + ...
        p.m(2)*(Jv2'*Jv2) + p.Iz(2)*(Jw2'*Jw2) + ...
        p.m(3)*(Jv3'*Jv3) + p.Iz(3)*(Jw3'*Jw3);
end

% ─────────────────────────────────────────────────────────────────
function C = coriolis_matrix(q, dq, p)
%CORIOLIS_MATRIX  Coriolis & centrifugal matrix via Christoffel symbols.
%
%   C_ij = Σ_k  Γ_ijk · q'_k
%
%   Christoffel symbol of first kind:
%       Γ_ijk = ½ ( ∂M_ij/∂q_k  +  ∂M_ik/∂q_j  −  ∂M_jk/∂q_i )
%
%   ∂M/∂q computed by central-difference numerical differentiation.
%   Property: Ṁ − 2C is skew-symmetric (energy passivity).
%
    n    = 3;
    eps  = 1e-7;
    dMdq = zeros(n, n, n);   % dMdq(:,:,k) = ∂M/∂q_k

    for k = 1:n
        qp = q;  qp(k) = q(k) + eps;
        qm = q;  qm(k) = q(k) - eps;
        dMdq(:,:,k) = (mass_matrix(qp,p) - mass_matrix(qm,p)) / (2*eps);
    end

    C = zeros(n, n);
    for i = 1:n
        for j = 1:n
            cij = 0;
            for k = 1:n
                Gamma_ijk = 0.5*(dMdq(i,j,k) + dMdq(i,k,j) - dMdq(j,k,i));
                cij = cij + Gamma_ijk * dq(k);
            end
            C(i,j) = cij;
        end
    end
end

% ─────────────────────────────────────────────────────────────────
function G = gravity_vector(q, p)
%GRAVITY_VECTOR  G(q) = ∂PE/∂q   (gradient of potential energy).
%
%   PE = Σᵢ mᵢ·g·y_ci
%
%   y_c1 = lc1·sin(q1)
%   y_c2 = L1·sin(q1) + lc2·sin(q1+q2)
%   y_c3 = L1·sin(q1) + L2·sin(q1+q2) + lc3·sin(q1+q2+q3)
%
    q1=q(1); q2=q(2); q3=q(3);
    c1   = cos(q1);
    c12  = cos(q1+q2);
    c123 = cos(q1+q2+q3);

    L1=p.L(1); L2=p.L(2);
    lc1=p.lc(1); lc2=p.lc(2); lc3=p.lc(3);

    G    = zeros(3,1);
    G(1) = p.g*( p.m(1)*lc1*c1 + ...
                 p.m(2)*(L1*c1 + lc2*c12) + ...
                 p.m(3)*(L1*c1 + L2*c12 + lc3*c123) );
    G(2) = p.g*( p.m(2)*lc2*c12 + ...
                 p.m(3)*(L2*c12 + lc3*c123) );
    G(3) = p.g*( p.m(3)*lc3*c123 );
end

% ─────────────────────────────────────────────────────────────────
function PE = potential_energy(q, p)
%POTENTIAL_ENERGY  PE = Σ mᵢ·g·y_ci  [J]
    q1=q(1);  q2=q(2);  q3=q(3);
    s1   = sin(q1);
    s12  = sin(q1+q2);
    s123 = sin(q1+q2+q3);

    L1=p.L(1); L2=p.L(2);
    lc1=p.lc(1); lc2=p.lc(2); lc3=p.lc(3);

    yc1 = lc1*s1;
    yc2 = L1*s1  + lc2*s12;
    yc3 = L1*s1  + L2*s12 + lc3*s123;
    PE  = p.g*(p.m(1)*yc1 + p.m(2)*yc2 + p.m(3)*yc3);
end

% ─────────────────────────────────────────────────────────────────
function joints = forward_kinematics(q1, q2, q3, p)
%FORWARD_KINEMATICS  Direct kinematic positions via 2D homogeneous transforms.
%
%   Returns 2×4 matrix:  columns = [base | joint2 | joint3 | EE_tip]
%
%   Transform convention (each T is 3×3 SE(2)):
%       T01 — rotate by q1           at origin
%       T12 — translate L1, rotate q2
%       T23 — translate L2, rotate q3
%       Tip — translate L3 along link 3 axis
%
    c1=cos(q1);      s1=sin(q1);
    c12=cos(q1+q2);  s12=sin(q1+q2);
    c123=cos(q1+q2+q3); s123=sin(q1+q2+q3);

    L1=p.L(1); L2=p.L(2); L3=p.L(3);

    Px = [0;
          L1*c1;
          L1*c1 + L2*c12;
          L1*c1 + L2*c12 + L3*c123];

    Py = [0;
          L1*s1;
          L1*s1 + L2*s12;
          L1*s1 + L2*s12 + L3*s123];

    joints = [Px'; Py'];   % 2×4
end




