clear;
%质心间距
L1=1;
L2=1;
L3=1;
L4=1;
m1 = 1;
m2 = 1;
m3 = 1;
m4 = 1;
m5 = 1;
g = 9.8;
r=0.1;
I_x = [1;
    1;
    1;
    1;];
I_z = [1;
        1;
        1;
        1];
syms x(t) theta1(t) theta2(t) theta3(t) theta4(t) theta5(t) theta1_1(t) theta2_2(t) theta3_3(t) theta4_4(t) theta5_5(t)
theta1v = diff(theta1_1 - theta1,t);
theta2v= diff(theta2_2 - theta2,t);
theta3v= diff(theta3_3 - theta3,t);
theta4v= diff(theta4_4 - theta4,t);
theta5v= diff(theta5_5 - theta5,t);
v = diff(x,t);
start_pos = sym(zeros(5,3));
end_pos = sym(zeros(5,3));
middle_val = sym(zeros(6,1));
middle_val(1) = calc(L2,theta2);
middle_val(2) = calc(L3,theta3);
middle_val(3) = calc(L4,theta4);
middle_val(4) = calc(L2,theta2_2);
middle_val(5) = calc(L3,theta3_3);
middle_val(6) = calc(L4,theta4_4);
start_pos(1,:) = [0 0 0];
start_pos(2,:) = start_pos(1,:) + [0 L1*sin(theta1) L1*cos(theta1)];
start_pos(3,:) = start_pos(2,:) + [-middle_val(1)*tan(theta5) middle_val(1) middle_val(1)/tan(theta5)];
start_pos(4,:) = start_pos(3,:) + [-middle_val(2)*tan(theta5) middle_val(2) middle_val(2)/tan(theta5)];
start_pos(5,:) = start_pos(4,:) + [-middle_val(3)*tan(theta5) middle_val(3) middle_val(3)/tan(theta5)];

end_pos(1,:) = [0 x 0];
end_pos(2,:) = end_pos(1,:) + [0 0 L1];
end_pos(3,:) = end_pos(2,:) + [-middle_val(4)*tan(theta5_5) middle_val(4) middle_val(4)/tan(theta5_5)];
end_pos(4,:) = end_pos(3,:) + [-middle_val(5)*tan(theta5_5) middle_val(5) middle_val(5)/tan(theta5_5)];
end_pos(5,:) = end_pos(4,:) + [-middle_val(6)*tan(theta5_5) middle_val(6) middle_val(6)/tan(theta5_5)];

distance = end_pos-start_pos;
pot_v_2 = sym(zeros(5,1));
for i = 1:5
    pot_v_2(i) = diff(distance(i,1),t)^2 + diff(distance(i,2),t)^2 + diff(distance(i,3),t)^2;
end

Ek_tans = 1/2*(m1*pot_v_2(1) + m2*pot_v_2(2) +m3*pot_v_2(3) +m4*pot_v_2(4) +m5*pot_v_2(5));
Ek_roll = 1/2*(I_x(1)*theta1v^2+I_x(2)*theta2v^2+I_x(3)*theta3v^2+I_x(4)*theta4v^2) +1/2*theta5v^2*(I_z(1)+I_z(2)+I_z(3)+I_z(4));
whole_EK = Ek_roll+Ek_tans;


V = g*(m1*end_pos(1,3)+m2*end_pos(2,3)+m3*end_pos(3,3)+m1*end_pos(4,3)+m1*end_pos(4,3)+m5*end_pos(5,3));
L = whole_EK- V;

q = sym(zeros(6,1));
dq = sym(zeros(6,1));
q(1) = x;
q(2) = theta1_1-theta1;
q(3) = theta2_2-theta2;
q(4)=theta3_3-theta3;
q(5)=theta4_4-theta4;
q(6)=theta5_5-theta5;

dq(1) = v;
dq(2) = theta1v;
dq(3) = theta2v;
dq(4) =theta3v;
dq(5) =theta4v;
dq(6) = theta5v;

ddq = diff(dq,t);
syms T1 T2 T3 T4 T5
u = [-T1/r;
      T2;
      T3;
      T4;
      T5;
      0];
%p = jacobian(L, dq');  % jacobian(L, dq')等价于dL/d(dq)的转置，需转置为向量
% p = p';  % 转为列向量
% 
% % 2. 计算dp/dt（全导数），并替换d(dq)/dt为ddq
% dp_dt = diff(p, t);  % 对t求全导数
% % 替换所有dq的导数为ddq（因diff(dq_i, t) = ddq_i）
% for i = 1:10
%     dp_dt = subs(dp_dt, diff(dq(i), t), ddq(i));
% end
% 
% % 3. 计算dL/dq（10×1）
% dL_dq = jacobian(L, q');
% dL_dq = dL_dq';
% 
% % 4. 整理拉格朗日方程：dp_dt - dL_dq = u，解出ddq
% eq = dp_dt - dL_dq == u;
% ddq_sol = solve(eq, ddq);  % 解出ddq关于q, dq, u的表达式
% ddq_sol = cell2mat(struct2cell(ddq_sol));  % 转为列向量

% dL_qd1 = diff(L,v); 
% dL_qd2 = diff(L,theta1v); 
% dL_qd3 = diff(L,theta2v); 
% dL_qd4 = diff(L,theta3v); 
% dL_qd5 = diff(L,theta4v); 
% dL_qd6 = diff(L,theta5v); 
