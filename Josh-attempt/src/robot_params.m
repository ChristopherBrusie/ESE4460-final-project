function p = robot_params()
%ROBOT_PARAMS Parameters for ESE446 3R planar robot.

p.L = [4; 3; 2];
p.m = [20; 15; 10];
p.Iz = [0.5; 0.2; 0.1];
p.lc = p.L/2;
p.g = 9.81;
p.friction = [0; 0; 0];       % frictionless — not specified in assignment
p.q0 = deg2rad([10; 20; 30]);
p.dq0 = [0; 0; 0];

end
