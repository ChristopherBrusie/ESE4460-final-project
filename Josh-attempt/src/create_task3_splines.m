function [pp1, pp2, pp3] = create_task3_splines()
%CREATE_TASK3_SPLINES Clamped splines with zero start/end slopes.

t = [0 2 4 6 8];
qdeg = [  0   30   45 150 180;
          0  -10  130  10   0;
         90   70  -85  70 -90];

q = deg2rad(qdeg);
pp1 = spline(t, [0 q(1,:) 0]);
pp2 = spline(t, [0 q(2,:) 0]);
pp3 = spline(t, [0 q(3,:) 0]);

end
