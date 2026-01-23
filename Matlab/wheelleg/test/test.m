% L1 = 282/1000; %ED
% L2 = 285.6/1000;%AD
% L3 = 292.81/1000;%BC
% L4 = 130.67/1000;%AB
% L23 = 57.02/1000;%DC
% syms alpha F
% angle_BAD = pi/4 - alpha;
% L_BD = sqrt(L2.^2 + L4.^2 -2*L2*L4*cos(angle_BAD));
% middle_val1 = ((L_BD).^2 + L2.^2 - L4.^2) / (2*L_BD*L2);
% middle_val2 = ((L_BD).^2 + L23.^2 - L3.^2) / (2*L_BD*L23);
% belta = pi+alpha - acos(middle_val1) - acos(middle_val2);
% F1 = F*cos(belta);
% F2 = F*sin(belta);
% F3 = (F1*(L1+L23)/2)/((L1+L23)/2-L23);
% theta = pi/2 - belta + alpha;
% F1_T = F3 *sin(theta);
% F2_T = F2*sin(theta);
% F_T = F2_T - F1_T;
% T = - F_T/L2;
% disp(vpa(T));






syms alpha  x y  
L1 = 282/1000; %ED
L2 = 285.6/1000;%AD
L3 = 292.81/1000;%BC
L4 = 130.67/1000;%AB
L23 = 57.02/1000;%DC

angle_BAD = pi/4 - alpha;
L_BD = sqrt(L2.^2 + L4.^2 -2*L2*L4*cos(angle_BAD));
middle_val1 = ((L_BD).^2 + L2.^2 - L4.^2) / (2*L_BD*L2);
middle_val2 = ((L_BD).^2 + L23.^2 - L3.^2) / (2*L_BD*L23);

belta = pi+alpha - acos(middle_val1) - acos(middle_val2);
eq1 = L2*cos(alpha) - L1*cos(belta) == x;
eq2 = L2*sin(alpha) - L1*sin(belta) == y;

sol = solve(eq1,eq2,x,y);
result = vpa(sol.y,6);
result2 = vpa(sol.x,6);
output = [sol.x,
        sol.y];
%alpha = -pi/8;
%my_x=0.2856*cos(alpha) + 0.282*cos(alpha + acos((8.76885*(0.0746387*cos(alpha - 0.785398) - 0.0161556))/(0.098642 - 0.0746387*cos(alpha - 0.785398))^(1/2)) + acos((1.7507*(0.0746387*cos(alpha - 0.785398) - 0.163135))/(0.098642 - 0.0746387*cos(alpha - 0.785398))^(1/2)))
%my_y=0.2856*sin(alpha) + 0.282*sin(alpha + acos((8.76885*(0.0746387*cos(alpha - 0.785398) - 0.0161556))/(0.098642 - 0.0746387*cos(alpha - 0.785398))^(1/2)) + acos((1.7507*(0.0746387*cos(alpha - 0.785398) - 0.163135))/(0.098642 - 0.0746387*cos(alpha - 0.785398))^(1/2)))

% disp(my_x);
% disp(my_y);
my_pos = forwardKinematics(L1,L2,L3,L4,L23,alpha);
disp("my_x=");
disp(vpa(my_pos.x,6));
disp("my_y=");
disp(vpa(my_pos.y,6));