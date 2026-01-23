%一二象限角度为正，三四象限角度为负
%功能：用于求关节执行角度范围

function result = angle_calc(L1,L2,L3,L4,L23)
    Lengths = [L2 L3 L4 L23];
    s = min(Lengths);
    l = max(Lengths);
    %四连杆Grashof定律
    pqSum = sum(Lengths) - s - l;%另外两杆长
    isReasonable =  (s+l) <= pqSum;%是否满足四连杆结构
    alphaRanges = [] ;%存储角度范围
    if isReasonable
        syms my_alpha
        B = [L4*cos(pi/4),L4*sin(pi/4)];
        D = [L2*cos(my_alpha),L2*sin(my_alpha)];
        L_BD = norm(D-B);%BD与alpha的关系式
        L_AC_min = abs(L3 - L23);
        L_AC_max = L3+L23;
        eq1 = L_BD ==L_AC_max;
        eq2 = L_BD ==L_AC_min;
        alpha1 = solve(eq1,my_alpha);
        alpha2 = solve(eq2,my_alpha);
        alphaRanges = [alpha1,alpha2];
        result.angle_min = alpha1;
        result.angle_max = alpha2;
        disp('alpha的取值范围(弧度):');
        disp(alphaRanges);
        disp('alpha的取值范围(角度):');
        disp(double(rad2deg(alphaRanges)));
    else 
        disp('机构不符合');
        return;
    end
end
