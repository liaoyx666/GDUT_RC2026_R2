% 测试脚本部分
clc;
clear all;
close all;
h = 0.001;
h0 = 10 * h;
r0 = 20;
tend = 10;
t = 0:h:tend;
v = sin(t);
dv = cos(t);

x1 = zeros(size(t));
x2 = zeros(size(t));

for k = 1:length(t)
   fh = fhan((x1(k) - v(k)), x2(k), r0, h0);
   if k < length(t)
       x1(k+1) = x1(k) + h * x2(k);
       x2(k+1) = x2(k) + h * fh;
   end
end

figure(1)
plot(t, v, 'r--', t, x1, 'b-', 'linewidth', 2)
xlabel("时间t/s")
ylabel("v,x1")
legend("v", "x1")
grid on

figure(2)
plot(t, dv, 'r--', t, x2, 'b-', 'linewidth', 2)
xlabel("时间t/s")
ylabel("dv,x2")
legend("dv", "x2")
grid on

% 局部函数定义（必须放在脚本后面）
function fh = fhan(x1, x2, r0, h0)
   d = r0 * h0 * h0;
   a0 = h0 * x2;
   y = x1 + a0;
   a1 = sqrt(d * (d + 8 * abs(y)));
   a2 = a0 + sign(y) * (a1 - d) / 2;
   sy = (sign(y + d) - sign(y - d)) / 2;
   a = (a0 + y - a2) * sy + a2;
   sa = (sign(a + d) - sign(a - d)) / 2;
   fh = -r0 * (a / d - sign(a)) * sa - r0 * sign(a);
end


