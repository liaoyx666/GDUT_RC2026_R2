function result = poly_calc(max_h,min_h,gep_size,segmentations,ploy_order)
    hs = min_h : gep_size :max_h;
    ks = zeros(length(hs),4);
    num = length(hs);
    for i = 1:num
        %ks(i,:) = LQR_calc(whole_pole_m+Body_M,Wheel_m,g,hs(i));
        ks(i,:) = LQR_calc2(Wheel_m,whole_pole_m+Body_M,g,hs(i),Wheel_R,I);
    end
    P = zeros(4, poly_order+1);
    for i = 1:4
        P(i,:) = polyfit(hs, ks(:,i), poly_order); 
    end
    y_fit1 = polyval(P(1,:),hs);
    y_fit2 = polyval(P(2,:),hs);
    y_fit3 = polyval(P(3,:),hs);
    y_fit4 = polyval(P(4,:),hs);
    figure(2);
    plot(hs,ks(:,1),'o',hs,y_fit1,'-',hs,ks(:,2),'o',hs,y_fit2,'-',hs,ks(:,3),'o',hs,y_fit3,'-',hs,ks(:,4),'o',hs,y_fit4,'-');
    legend('原始数据','拟合曲线');
end

