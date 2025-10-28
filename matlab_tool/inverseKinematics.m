function result = inverseKinematics(X, Y)
    L1 = 235; %ED
    L2 = 238;%AD
    L3 = 244;%BC
    L4 = 108.89;%AB
    L23 = 57;%DC

    phi = atan(-Y/X);
    R = sqrt(X^2 +Y^2);
    middle_val1 = 2*L2*R;
    middle_val2 = L2^2 -L1^2 + R^2;

    acos_arg = middle_val2 / middle_val1;
    
    angle_out1 = phi + acos(acos_arg);
    angle_out2 = phi - acos(acos_arg);
    disp(double(rad2deg(angle_out1)));
    disp(double(rad2deg(angle_out2)));

    if -0.089>angle_out1 >1.226
        angle_out = -angle_out1;
    else
        angle_out = -angle_out2;
    end

    disp('角度制输出');
    disp(double(rad2deg(angle_out)));
    disp('弧度制输出');
    disp(double(angle_out));

    A = [0,0];
    B = [L4*cos(pi/4),L4*sin(pi/4)];
    D = [L2*cos(angle_out),L2*sin(angle_out)];
    E = [X,Y]; 
    %C可能有多个解
    syms x y
    eq1 = (x - D(1)).^2 + (y - D(2)).^2 == L23.^2;
    eq2 = (x - B(1)).^2 + (y - B(2)).^2 == L3.^2;
    sol = solve(eq1,eq2,x,y);
    C = [double(sol.x),double(sol.y)];
    disp(C);

    
    result.A=A;
    result.B=B;
    result.C=C;
    result.D=D;
    result.E=E;
    % result.angle_out = (angle_out*180)/pi;
    % fprintf('alpha1 = %.2f\n', (angle_out1*180)/2*pi);
    % fprintf('alpha2 = %.2f\n', (angle_out2*180)/2*pi);
    % fprintf('x=%.2f ,y=%.2f\n',X,Y);

end


