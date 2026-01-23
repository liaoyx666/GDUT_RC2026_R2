clear;
L1 = 282/1000; %ED
L2 = 285.6/1000;%AD
L3 = 292.81/1000;%BC
L4 = 130.67/1000;%AB
L23 = 57.02/1000;%DC
m_L1_23 = 0.442;
m_L1 = m_L1_23*L1/(L1+L23);
m_L23 = m_L1_23*L23/(L1+L23);
m_L2 = 0.498;
m_L3 = 0.536;
whole_pole_m = m_L1_23 +m_L3+m_L2;
Wheel_R = 60/1000;
Wheel_W = 27/1000;
Wheel_m = 0.287;%kg
I = 0.870168809;%腿转动惯量（Kg*m^2）
Body_M = 10.113 - m_L1_23*2-m_L2*2 - m_L3*2 - 2*Wheel_m;%kg

m = whole_pole_m+Body_M;
M = Wheel_m;
g = 9.8;
L = 0.3;

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

Q = diag([1 1 1 1]);%x q x_d q_d
Q_end = diag([2 2 2 2]);%最后一次预测的权重
R =1;

Step = 5;%预测步长

unit = diag([1 1 1 1]);
whole_M =[unit;
          A;
          A.^2;
          A.^3;
          A.^4];
a = zeros(4,4);
whole_Q = [ Q a a a a;
            a Q a a a;
            a a Q a a;
            a a a Q a;
            a a a a Q];
whole_R = diag([R R R R R]);
a1 = zeros(4,1);
whole_C =  [a1          a1             a1      a1   a1;
            B           a1             a1      a1   a1;
            A*B         B              a1      a1   a1;
            (A.^2)*B    A*B            B       a1   a1;
            (A.^3)*B   (A.^2)*B        A*B     B    a1];
syms u1 u2 u3 u4 u5
start_state = [1;
        1;
        1;
        1];
whole_u = [u1;
           u2;
           u3;
           u4;
           u5];

G = whole_M.'*whole_Q*whole_M;
E = whole_C.'*whole_Q*whole_M;
H = whole_C.'*whole_Q*whole_C+whole_R;
J = start_state.'*G*start_state + 2*start_state.'*E*whole_u +whole_u.'*H*whole_u;






