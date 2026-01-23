L1 = 282/1000; %ED
L2 = 285.6/1000;%AD
L3 = 292.81/1000;%BC
L4 = 130.67/1000;%AB
L23 = 57.02/1000;%DC
angle_range = angle_calc(L1,L2,L3,L4,L23);

hs = -1.11:0.001:-0.17;
ks = zeros(length(hs),2);
num = length(hs);
for i = 1:num
    ks(i,:) = VMC_test(hs(i));
end
poly_order = 3; 
P = zeros(2, poly_order+1);

for i = 1:2
     P(i,:) = polyfit(hs, ks(:,i), poly_order); 
end

y_fit1 = polyval(P(1,:),hs);
y_fit2 = polyval(P(2,:),hs);
figure(2);
plot(hs,ks(:,1),'o',hs,y_fit1,'-',hs,ks(:,2),'o',hs,y_fit2,'-');
legend('原始数据','拟合曲线');

% 输出C代码（自动生成）
Knames = ["K1","K2"];
for i = 1:2
    coefs = P(i,:);
    % 拼接输出字符串
    c_code = sprintf('J_T=', Knames(i));
    for j = 1:(poly_order+1)
        pow = poly_order + 1 - j;
        if j > 1 && coefs(j) >= 0
            c_code = [c_code '+'];
        end
        if pow == 0
            c_code = [c_code sprintf('%.6f', coefs(j))];
        elseif pow == 1
            c_code = [c_code sprintf('%.6f*alpha', coefs(j))];
        else
            c_code = [c_code sprintf('%.6f*alpha^%d', coefs(j), pow)];
        end
    end
    c_code = [c_code '; '];
    disp(c_code)
end
