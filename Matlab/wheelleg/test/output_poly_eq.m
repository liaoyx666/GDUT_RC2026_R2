function result = output_poly_eq()
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
end