clear;
syms alpha
L1 = 282/1000; %ED
L2 = 285.6/1000;%AD
L3 = 292.81/1000;%BC
L4 = 130.67/1000;%AB
L23 = 57.02/1000;%DC
my_result = forwardKinematics(L1,L2,L3,L4,L23,alpha);
my_x = my_result.x;
my_y = my_result.y;
%my_y = 0.2856*sin(alpha) + 0.282*sin(alpha + acos((8.76885*(0.0746387*cos(alpha - 0.785398) - 0.0161556))/(0.098642 - 0.0746387*cos(alpha - 0.785398))^(1/2)) + acos((1.7507*(0.0746387*cos(alpha - 0.785398) - 0.163135))/(0.098642 - 0.0746387*cos(alpha - 0.785398))^(1/2)));
%my_x = 0.2856*cos(alpha) + 0.282*cos(alpha + acos((8.76885*(0.0746387*cos(alpha - 0.785398) - 0.0161556))/(0.098642 - 0.0746387*cos(alpha - 0.785398))^(1/2)) + acos((1.7507*(0.0746387*cos(alpha - 0.785398) - 0.163135))/(0.098642 - 0.0746387*cos(alpha - 0.785398))^(1/2)));

d_x = vpa(diff(my_x,alpha),6);
d_y = vpa(diff(my_y,alpha),6);
%雅可比矩阵
J = [d_x;
    d_y];
J_T = J.';
disp(vpa(J_T,6));
% T_out = J_T*F;