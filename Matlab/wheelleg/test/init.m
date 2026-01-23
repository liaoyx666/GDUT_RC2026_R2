clear;
Wall_H = 2;
Wall_W = 0.2;
Wall_Length = 2;

L1 = 282/1000; %ED
L2 = 285.6/1000;%AD
L3 = 292.81/1000;%BC
L4 = 130.67/1000;%AB
L23 = 57.02/1000;%DC
g = 9.8;
ground_h = 0.1;
Body_Wide = 600/1000;
Body_Height = 180/1000;
Body_Length = 380/1000;

m_L1_23 = 0.442;
m_L1 = m_L1_23*L1/(L1+L23);
m_L23 = m_L1_23*L23/(L1+L23);
m_L2 = 0.498;
m_L3 = 0.536;
whole_pole_m = m_L1_23 +m_L3+m_L2;
Wheel_R = 60/1000;
Wheel_W = 27/1000;
Wheel_m = 0.287;%kg
I = 0.870168809;%腿转动惯量（Kg*m^2）
Body_M = 10.113 - m_L1_23*2-m_L2*2 - m_L3*2 - 2*Wheel_m;%kg

% pole_L = 0.4;

% LQR_output = LQR_calc(whole_pole_m+Body_M/2,Wheel_m,g,pole_L);
% K = LQR_output;

angle_range = angle_calc(L1,L2,L3,L4,L23);
wheel_pos_max_angle = forwardKinematics(L1,L2,L3,L4,L23,angle_range.angle_max);
wheel_pos_min_angle = forwardKinematics(L1,L2,L3,L4,L23,angle_range.angle_min);
disp('polemax:');
disp(wheel_pos_max_angle);
disp('polemin:')
disp(wheel_pos_min_angle);
pole_range = [wheel_pos_min_angle.y,wheel_pos_max_angle.y];
disp('腿长范围:');
disp(pole_range);

angle_choose = angle_range.angle_min:0.01:angle_range.angle_max;
pot = zeros(length(angle_choose),2);
num1 = length(angle_choose);
for i = 1:num1
    result= forwardKinematics(L1,L2,L3,L4,L23,angle_choose(i));
    pot(i,1) = result.x;
    pot(i,2) = result.y;
end
figure(1);
plot(pot(:,1),pot(:,2),'o-','LineWidth',1.5,'MarkerFaceColor','b');
xlabel('X坐标');
ylabel('Y坐标');
title('正运动学计算的坐标点');
grid on;

 K= LQR_calc2(Wheel_m,whole_pole_m+Body_M,g,-0.3,Wheel_R,I);
 % A = K.A;
 % B =K.B;
 % C =K.C;
 % D = K.D;
 % disp(K);

%0.01~0.53
%hs = 0.01 : 0.01 :0.53;
hs = 0.01 : 0.001 :0.05;
ks = zeros(length(hs),4);
num = length(hs);
for i = 1:num
    %ks(i,:) = LQR_calc(whole_pole_m+Body_M,Wheel_m,g,hs(i));
    ks(i,:) = LQR_calc2(Wheel_m,whole_pole_m+Body_M,g,hs(i),Wheel_R,I);
end
poly_order = 3; 
P = zeros(4, poly_order+1);
for i = 1:4
     P(i,:) = polyfit(hs, ks(:,i), poly_order); 
end

y_fit1 = polyval(P(1,:),hs);
y_fit2 = polyval(P(2,:),hs);
y_fit3 = polyval(P(3,:),hs);
y_fit4 = polyval(P(4,:),hs);
figure(2);
plot(hs,ks(:,1),'o',hs,y_fit1,'-',hs,ks(:,2),'o',hs,y_fit2,'-',hs,ks(:,3),'o',hs,y_fit3,'-',hs,ks(:,4),'o',hs,y_fit4,'-');
legend('原始数据','拟合曲线');

% 输出C代码（自动生成）
Knames = ["K1", "K2", "K3", "K4"];
for i = 1:4
    coefs = P(i,:);
    % 拼接输出字符串
    c_code = sprintf('K=', Knames(i));
    for j = 1:(poly_order+1)
        pow = poly_order + 1 - j;
        if j > 1 && coefs(j) >= 0
            c_code = [c_code '+'];
        end
        if pow == 0
            c_code = [c_code sprintf('%.6f', coefs(j))];
        elseif pow == 1
            c_code = [c_code sprintf('%.6f*h', coefs(j))];
        else
            c_code = [c_code sprintf('%.6f*h^%d', coefs(j), pow)];
        end
    end
    c_code = [c_code '; '];
    disp(c_code)
end



