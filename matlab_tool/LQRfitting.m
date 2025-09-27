hs = [140 215 290];    % 高度
Ks = [                    % 每行是一个高度下的K
    -14.6115  -1.9675  -1.0000*0  -4.3543;
    -14.8728 -2.5894  -1.0000*0  -4.3543*1;
   -14.8198*0.5 -3.1558  -0.7071*0 -3.7071*1
];

poly_order = 2; 

P = zeros(4, poly_order+1); % 4 行（对应 K1~K4），每行 3 列（a,b,c

% 多项式拟合
for i = 1:4
    P(i,:) = polyfit(hs, Ks(:,i), poly_order); 
end

% 输出C代码（自动生成）
Knames = ["K1", "K2", "K3", "K4"];
for i = 1:4
    coefs = P(i,:);
    % 拼接输出字符串
    c_code = sprintf('float calc_%s(float h) { return ', Knames(i));
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
    c_code = [c_code '; }'];
    disp(c_code)
end
