% clear, clc, close all;
% radian = 180/pi; % radians to degrees
%关节大臂参数
%% MOD-DH参数
%连杆偏移
% d1 = 0;
% d2 = 0;
% d3 = 0;
% d4 = 0;
% d5 = 0;
% d6 = 0;
% %连杆长度
% a2 = 220;
% a3 = 267.23;
% %连杆扭角
% alpha1 = 0;
% alpha2 = -pi/2;
% alpha3 = 0;
% alpha4 = pi/2;
% alpha5 = -pi/2;
% alpha6 = pi/2;
% syms theta1 theta2 theta3 theta4 theta5 theta6
% %       theta  d        a        alpha     
% L1=Link([0   0          0         0;],'modified');
% L2=Link([0   0          0       -pi/2 ],'modified');
% L3=Link([0   0          306.07       0 ],'modified');
% L4=Link([0   0          279.35      pi/2; ],'modified');
% L5=Link([0   0          0        -pi/2  ],'modified'); L5.offset = pi/2;
% L6=Link([0   0          0        pi/2 ],'modified');
% robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','myrobot');
% robot.teach;
%SCARA参数
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

L2=Link([0 0 L(2) 0]); 
L3=Link([0 0 L(3) 0]); 
L4=Link([0 L(4) 0 -pi/2]);  
L5=Link([0 0 0 pi/2]); 
L6=Link([0 L(6) 0 0]); 
robot.qlim=[0 500;-90*pi/180 90*pi/180;-120*pi/180 120*pi/180];%关节限制 
robot=SerialLink([L1 L2 L3 L4 L5 L6]); 
% base_bias=[-200,-50,485];             %%这里已经调整好了 
% robot.base=transl(base_bias); 
%myT = scara_fkine(100,pi/2,pi/4,pi/2,pi/4,pi/2,robot);
%T=robot.fkine([100,pi/2,pi/4,pi/2,pi/4,pi/2]);
robot.teach;