function result = inverseKinematics(L1,L2,L3,L4,L23,X, Y)
    phi = atan(-Y/X);
    R = sqrt(X^2 +Y^2);
    middle_val1 = 2*L2*R;
    middle_val2 = L2^2 -L1^2 + R^2;
    acos_arg = middle_val2 / middle_val1;
    angle_out1 = -real(phi + acos(acos_arg));
    angle_out2 = -real(phi - acos(acos_arg));
    % E1 = forwardKinematics(L1,L2,L3,L4,L23,-angle_out1);
    % E2 = forwardKinematics(L1,L2,L3,L4,L23,-angle_out2);
    result.angle =[double(angle_out1),double(angle_out2)]; 
end


