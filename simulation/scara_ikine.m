%逆运动学
function [L1_new,q2,q3,q4,q5,q6] = scara_ikine(desired_pose,L)
radian = 180/pi; % radians to degrees
du = pi/180;     % degrees to radians
    q5 = desired_pose.pitch;
    q6 = desired_pose.roll;

    %计算位置偏移
    L1_new = desired_pose.z -L(4) + L(6)*cos(q5);

    %计算关节2、3的旋转角度和关节1的伸长高度
    d = sqrt(desired_pose.x^2+desired_pose.y^2);
    
    alpha = atan2(desired_pose.y,desired_pose.x);
    beta = acos((L(2)^2 + d^2 - L(3)^2) / (2 * L(2) * d));
    q2 = alpha+beta;
    gamma = acos((L(2)^2+L(3)^2-d^2)/(2*L(2)*L(3)));
    q3 = pi -gamma;
  
    %旋转关节4补偿偏移
    q4 = -pi/2 -q2 -q3;
end

