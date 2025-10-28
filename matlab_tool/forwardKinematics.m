function result = forwardKinematics(alpha)
    if -0.089>alpha>-1.226
        L1 = 235; %ED
        L2 = 238;%AD
        L3 = 244;%BC
        L4 = 108.89;%AB
        L23 = 57;%DC

        angle_BAD = pi/4 - alpha;
        L_BD = sqrt(L2.^2 + L4.^2 -2*L2*L4*cos(angle_BAD));
        middle_val1 = ((L_BD).^2 + L2.^2 - L4.^2) / (2*L_BD*L2);
        middle_val2 = ((L_BD).^2 + L23.^2 - L3.^2) / (2*L_BD*L23);
        belta = pi+alpha - acos(middle_val1) - acos(middle_val2);
        syms x y
        eq1 = L2*cos(alpha) - L1*cos(belta) == x;
        eq2 = L2*sin(alpha) - L1*sin(belta) == y;
        sol = solve(eq1,eq2,x,y);
        output = [double(sol.x),double(sol.y)];
        disp(output);
    else
        disp('角度错误')
    end
end