function angle = detal_test2(target_position)  
DELTA_R = 300;       % 半径或类似参数  
DELTA_L_ARM = 455;   % 臂的长度  
DELTA_r = 100;       % 另一个半径或偏差  
DELTA_A_ARM = 1100;   % 可能代表臂的某个补偿值  
PI = 3.141592653589793; % 圆周率  
%静平台端点与x轴的夹角
fail1 = 0;  
fail2 = 2 * PI / 3;  
fail3 = 4 * PI / 3;   
x =  target_position(1)
y =  target_position(2)
z =  target_position(3)

A1 =(DELTA_R - DELTA_r)^2+ x*x + y*y + z*z + (DELTA_L_ARM^2- DELTA_A_ARM^2) + 2*(DELTA_L_ARM - DELTA_R + DELTA_r)*(x*cos(fail1)+y*sin(fail1)) - 2 * DELTA_L_ARM *(DELTA_R - DELTA_r)
B1 = 4 * z * DELTA_L_ARM
C1 = (DELTA_R - DELTA_r)^2+ x*x + y*y + z*z + (DELTA_L_ARM^2- DELTA_A_ARM^2)+ 2*(DELTA_r - DELTA_L_ARM - DELTA_R )*(x*cos(fail1)+y*sin(fail1))- 2 * DELTA_L_ARM *(DELTA_R - DELTA_r)

%万能公式
t1 = -B1 + sqrt(B1^2 - 4 * A1 * C1) / (2 * A1); % 计算 t  

 theta1 = 2*atan(t1) 

 A2 =(DELTA_R - DELTA_r)^2+ x*x + y*y + z*z + (DELTA_L_ARM^2- DELTA_A_ARM^2)+ 2*(DELTA_L_ARM - DELTA_R + DELTA_r)*(x*cos(fail2)+y*sin(fail2)) - 2 * DELTA_L_ARM *(DELTA_R - DELTA_r)
B2 = 4 * z * DELTA_L_ARM
C2 = (DELTA_R - DELTA_r)^2+ x*x + y*y + z*z + (DELTA_L_ARM^2- DELTA_A_ARM^2)+ 2*(DELTA_r - DELTA_L_ARM - DELTA_R )*(x*cos(fail2)+y*sin(fail2))- 2 * DELTA_L_ARM *(DELTA_R - DELTA_r)

%万能公式
t2 = -B2 + sqrt(B2 ^2 - 4 * A2 * C2) / (2 * A2); % 计算 t  

 theta2 = 2*atan(t2) 


 A3 =(DELTA_R - DELTA_r)^2+ x*x + y*y + z*z + (DELTA_L_ARM^2- DELTA_A_ARM^2) + 2*(DELTA_L_ARM - DELTA_R + DELTA_r)*(x*cos(fail3)+y*sin(fail3))- 2 * DELTA_L_ARM *(DELTA_R - DELTA_r)
B3 = 4 * z * DELTA_L_ARM
C3 = (DELTA_R - DELTA_r)^2+ x*x + y*y + z*z + (DELTA_L_ARM^2- DELTA_A_ARM^2)+ 2*(DELTA_r - DELTA_L_ARM - DELTA_R )*(x*cos(fail3)+y*sin(fail3))- 2 * DELTA_L_ARM *(DELTA_R - DELTA_r)

%万能公式
t3 = -B3 + sqrt(B3 ^2 - 4 * A3 * C3) / (2 * A3); % 计算 t  

 theta3 = 2*atan(t3) 
 
 angle = [theta1,theta2,theta3]
end