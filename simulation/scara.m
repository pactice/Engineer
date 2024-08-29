clear, clc, close all;
radian = 180/pi; % radians to degrees
du = pi/180;     % degrees to radians
%% 生成机械臂 
% 杆长 （这里的杆长已经是调整好的了） 
l1=0; 
l2=215; 
l3=215; 
l4=200; % 几个轴都有z偏置，在此视为0，即一开始的基准平面是过5轴的转轴的平面 
l5=0;  
l6=100; 
L=[l1,l2,l3,l4,l5,l6]; 
 
% DH参数  [theta d a alpha] 
L1=Link([0 0 L(1) 0]); L1.qlim = [0 500];
L1.sigma = 1; 
L2=Link([0 0 L(2) 0]); 
L3=Link([0 0 L(3) 0]); 
L4=Link([0 L(4) 0 -pi/2]);  
L5=Link([0 0 0 pi/2]); 
L6=Link([0 L(6) 0 0]); 
robot.qlim=[0 500;-90*pi/180 90*pi/180;-120*pi/180 120*pi/180];%关节限制 
robot=SerialLink([L1 L2 L3 L4 L5 L6]); 
base_bias=[-200,-50,485];             %%这里已经调整好了 
robot.base=transl(base_bias); 
myT = scara_fkine(100,pi/2,pi/4,pi/2,pi/4,pi/2,robot);
T=robot.fkine([100,pi/2,pi/4,pi/2,pi/4,pi/2]);


% 定义所需的姿态参数desired_pose，包括位置和姿态
% 定义desired_pose结构体
desired_pose.x = 400;
desired_pose.y = 400;
desired_pose.z = 500;
desired_pose.pitch = pi/3;
desired_pose.roll = pi/3;
desired_pose.yaw = pi/4;


% 定义机械臂的长度参数L，包括每个关节的长度
L=[l1,l2,l3,l4,l5,l6]; 

% 调用逆运动学解算函数
[L1_new,q2,q3,q4,q5,q6] = scara_ikine(desired_pose, L);

% 初始化工作空间矩阵
workspace = zeros(3000, 3);

for index = 1:3000
    % 生成随机的关节角度
    q1 = rand() * 500;  % 关节1的角度范围[0, 500]
    q2 = (rand() - 0.5) * pi;  % 关节2的角度范围[-pi/2, pi/2]
    q3 = (rand() - 0.5) * pi;  % 关节3的角度范围[-pi/2, pi/2]
    q4 = (rand() - 0.5) * pi;  % 关节4的角度范围[-pi/2, pi/2]
    q5 = (rand() - 0.5) * pi;  % 关节5的角度范围[-pi/2, pi/2]
    q6 = (rand() - 0.5) * pi;  % 关节6的角度范围[-pi/2, pi/2]
    
    % 计算末端执行器位置
    end_effector_pos = robot.fkine([q1, q2, q3, q4, q5, q6]);
    
    % 仅保留末端执行器位置的前三个坐标
    workspace(index, :) = end_effector_pos(1:3);
end

% 绘制工作空间
scatter3(workspace(:,1), workspace(:,2), workspace(:,3), 'filled');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('机械臂工作空间（蒙特卡洛法，3000个随机点）');

% % scara机器人工具箱建模
% clc
% clear ;
% L1=Link([0 0 0 0 0],'modified');
% L2=Link([0 0 225 0 0],'modified'); 
% L3=Link([0 0 275 0 1],'modified');%移动关节最后一个参数为1
% L4=Link([0 0 0 0 0],'modified'); 
% Robot=SerialLink([L1 L2 L3 L4],'name','Scara');
% Robot.qlim=[-132*pi/180 132*pi/180;-145*pi/180 145*pi/180;0 200;-355*pi/180 355*pi/180];%KR 6 R500 Z200关节限制
% q1=[pi/2,pi/3,100,pi/3];
% %Robot.plot([0 0 0 0], 'workspace',[-400 400 -400 400 -300 300]);%空间范围定义
% T=Robot.fkine(q1);
% myT=scara_fkine(pi/2,pi/3,100,pi/3,Robot)
% Robot.plotopt = {'workspace',[-500 500 -500 500 -500 500],'tilesize',300};
%Robot.teach();
% disp(T)