function MPC_Nextstate = MPC_calc(m, M, g, L, x_now,u)
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
    MPC_Nextstate = A*x_now +B*u;%4*1
    %y = C*state+D*u;%4*1
    
end