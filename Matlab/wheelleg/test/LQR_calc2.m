function result = LQR_calc2(M,m,g,h,r,I)
    D = (M+m)*(m*h^2 + I) - m^2*h^2;

    A = [0 1 0 0; ((M+m)*m*g*h)/D 0 0 0;0 0 0 1;-(m^2*g*h^2)/D 0 0 0];
    B = [0; -(m*h)/(D*r);0;1/((M+m)*r)-(m^2*h^2)/((M+m)*D*r)];
    Q = diag([1 1 1 1]);%q q_d x x_d
    R = 33;
    result = lqr(A, B, Q, R);

    % Z = M + m + i / (r * r);
    % D = Z * (I + M * h * h) - M * M * h * h;
    % 
    % a = (M * g * h * Z) / D;
    % b = -(M * M * h * h * g) / D;
    % c = -(M * h) / (D * r);
    % d = 1 / (Z * r) - (M * M * h * h) / (D * Z * r);
    % A =[ 0       1      0      0;
    %      a       0      0      0;
    %      0       0      0      1;
    %      b       0      0      0];
    % B =[ 0;
    %      c;
    %      0;
    %      d];
    % result = lqr(A, B, Q, R)
end