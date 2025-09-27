g=9.874;
m = 0.448 * 2; %腿部质量之和(kg)
M = 15.607; %车体质量(kg)
L = 407.8403830603180400345208157179*1e-3;%转轴到车身质心距离
R = 0.06;%车轮半径
I = 1/2 * m * R^2;%两车轮转动惯量之和 
J = 3.683598685 ; %车身转动惯量
syms x_dd phi_dd;
syms xb_dd yb_dd theta theta_d theta_dd;
syms C P H V;
eq1 = m * x_dd == P - H;
eq2 = I * phi_dd == C - P * R;
eq3 = x_dd == phi_dd * R;
eq4 = M * xb_dd == H;
eq5 = M * yb_dd == V - M * g;
eq6 = J * theta_dd == V * L * sin(theta) - H * L * cos(theta) - C;
eq7 = xb_dd == x_dd + L*cos(theta) * theta_dd - L*sin(theta) * theta_d^2;
eq8 = yb_dd == -L * sin(theta) * theta_dd  - L * cos(theta) * theta_d^2;
vars = [x_dd, phi_dd, xb_dd, yb_dd, theta_dd, P, H, V];
sol = solve([eq1, eq2, eq3, eq4, eq5, eq6, eq7, eq8], vars);
a23 = double(subs(diff(sol.x_dd, theta), [theta, theta_d, C], [0, 0, 0]));
a24 = double(subs(diff(sol.x_dd, theta_d), [theta, theta_d, C], [0, 0, 0]));
a43 = double(subs(diff(sol.theta_dd, theta), [theta, theta_d, C], [0, 0, 0]));
a44 = double(subs(diff(sol.theta_dd, theta_d), [theta, theta_d, C], [0, 0, 0]));
b21 = double(subs(diff(sol.x_dd, C), [theta, theta_d, C], [0, 0, 0]));
b41 = double(subs(diff(sol.theta_dd, C), [theta, theta_d, C], [0, 0, 0]));
A = [0   1   0   0;
     0   0  a23  a24;
     0   0   0   1;
     0   0  a43  a44];
B = [ 0 ;
     b21;
      0 ;
     b41];
Q_diag = [20 18 14 22]; %x x_d theta theta_d
Q = diag(Q_diag);
R = 1;
K = lqr(A, B, Q, R);
disp("Q:");
disp(Q_diag);
disp("R:");
disp(R);
disp("K:");
disp(K);
