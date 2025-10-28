function my_test(X,Y)
    % dot = linspace(-1.226,-0.089,100);%将弧度范围分成100份
    % my_dot = zeros(length(dot),2);%存储正解坐标
    % my_dot2 = zeros(length(dot),2);%存储逆解角度转正解的坐标
    % angle = zeros(length(dot),1);%存储角度
    % for i=1:length(dot)
    %     my_dot(i,:) = forwardKinematics(dot(i));
    %     angle(i) = inverseKinematics(my_dot(i,1),my_dot(i,2));
    %     my_dot2(i,:)= forwardKinematics(angle(i));
    % end
    % % disp('first');
    % disp(my_dot);
    % disp('second');
    % disp(my_dot2);
    % plot(my_dot(:,1),my_dot(:,2),'-b','LineWidth',1.2);
    %plot(my_dot2(:,1),my_dot2(:,2),'-b','LineWidth',1.2);

    angle = inverseKinematics(X,Y);
    dot = forwardKinematics(angle);
    angle2 = inverseKinematics(dot(1),dot(2));
    dot2 = forwardKinematics(angle2);
    disp(angle);
    disp(angle2);
    disp(dot);
    disp(dot2);
end
