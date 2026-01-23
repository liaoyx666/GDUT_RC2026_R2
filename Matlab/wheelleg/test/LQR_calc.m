function LQR_output = LQR_calc(m,M,g,L)
    a32 = (m*g)/M;
    a42 = ((m+M)*g)/(M*L);
    b3 = 1/M;
    b4 = 1/(M*L);
    A =[0 0 1 0;
        0 0 0 1;
        0 a32 0 0;
        0 a42 0 0];
    B = [0;
         0;
         b3;
         b4];
    C = [1 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
    D = [0;
        0;
        0;
        0];

    Q =diag([1 1 1 1]);%x q x_d q_d
    R = 1;
    K = lqr(A,B,Q,R);
    LQR_output = K;
    % % LQR_output.K = K;
    % LQR_output.A=A;
    % LQR_output.B=B;
    % LQR_output.C=C;
    % LQR_output.D=D;  
end