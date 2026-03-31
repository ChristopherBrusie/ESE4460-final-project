close all; clc; clear all



%% Task 1: Dynamics
m_link1 = 20; % mass
m_link2 = 15;
m_link3 = 10;

t1_0 = deg2rad(10); % initial joint angles (rad)
t2_0 = deg2rad(20);
t3_0 = deg2rad(30);



%% Visualize Arm

% just testing the visualization
t_test = linspace(0, 2*pi, 1000);
for i = 1:1000
    visualize_arm(t_test(i), t_test(i), t_test(i))
end



function visualize_arm(q1, q2, q3)
    % Define link lengths
    L1 = 4; L2 = 3; L3 = 2;


    % Base to Link 1 (Rotation at origin)
    T01 = [cos(q1), -sin(q1), 0;
           sin(q1),  cos(q1), 0;
           0,        0,       1];
       
    % Link 1 to Link 2 (Translate by L1, then rotate by q2)
    T12 = [cos(q2), -sin(q2), L1;
           sin(q2),  cos(q2), 0;
           0,        0,       1];
       
    % Link 2 to Link 3 (Translate by L2, then rotate by q3)
    T23 = [cos(q3), -sin(q3), L2;
           sin(q3),  cos(q3), 0;
           0,        0,       1];

    % 2. Calculate Global Positions
    % Points are defined as [0; 0; 1] in their local frames
    P0 = [0; 0; 1];                     % Base
    P1 = T01 * [0; 0; 1];               % Joint 1
    P2 = T01 * T12 * [0; 0; 1];         % Joint 2
    P3 = T01 * T12 * T23 * [0; 0; 1];   % End Effector (Translate by L3 from P2)
    
    % The end effector tip is L3 away from P3 along its local x-axis
    P_tip = T01 * T12 * T23 * [L3; 0; 1];

    % 3. Extract Coordinates for Plotting
    X = [P0(1), P1(1), P2(1), P3(1), P_tip(1)];
    Y = [P0(2), P1(2), P2(2), P3(2), P_tip(2)];

    % 4. Graphics Handling (Optimized for Simulink)
    persistent h;
    if isempty(h) || ~isgraphics(h)
        figure('Name', '3R Robot - Matrix Kinematics');
        h = plot(X, Y, '-o', 'LineWidth', 2, 'MarkerFaceColor', 'k');
        grid on; axis equal;
        limit = L1 + L2 + L3 + 0.5;
        axis([-limit limit -limit limit]);
    else
        set(h, 'XData', X, 'YData', Y);
    end
    drawnow;
end