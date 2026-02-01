function result = forwardKinematics(L1,L2,L3,L4,L23,alpha)
        angle_BAD = pi/4 - alpha;
        L_BD = sqrt(L2.^2 + L4.^2 -2*L2*L4*cos(angle_BAD));
        middle_val1 = ((L_BD).^2 + L2.^2 - L4.^2) / (2*L_BD*L2);
        middle_val2 = ((L_BD).^2 + L23.^2 - L3.^2) / (2*L_BD*L23);
        belta = pi+alpha - acos(middle_val1) - acos(middle_val2);
        %disp("smell leg to gound angle");
        %disp(belta);
        syms x y
        eq1 = L2*cos(alpha) - L1*cos(belta) == x;
        eq2 = L2*sin(alpha) - L1*sin(belta) == y;
        sol = solve(eq1,eq2,x,y);
        % output = [double(sol.x),double(sol.y)];
        result.x = real(sol.x);
        result.y = real(sol.y);
end