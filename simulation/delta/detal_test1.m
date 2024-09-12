% 定义常量 (请根据实际值设置这些常量)  
% DELTA_R = 0.1;       % 半径或类似参数  
% DELTA_L_ARM = 0.1;   % 臂的长度  
% DELTA_r = 0.04;       % 另一个半径或偏差  
% DELTA_A_ARM = 0.224;   % 可能代表臂的某个补偿值  
% PI = 3.141592653589793; % 圆周率  
DELTA_R = 300;       % 半径或类似参数  
DELTA_L_ARM = 200;   % 臂的长度  
DELTA_r = 150;       % 另一个半径或偏差  
DELTA_A_ARM = 400;   % 可能代表臂的某个补偿值  
PI = 3.141592653589793; % 圆周率  
%静平台端点与x轴的夹角
fail1 = 0;  
fail2 = 2 * PI / 3;  
fail3 = 4 * PI / 3;  
  
% 从控制器结构中获取电机偏移位置  
cita1 = 2 * PI /3;  
cita2 = 2 * PI /3;  
cita3 = 2 * PI /3;  

% 计算点 C1, C2, C3 的坐标  
point_C1 = [DELTA_R * cos(fail1), DELTA_R * sin(fail1), 0];  
point_C2 = [DELTA_R * cos(fail2), DELTA_R * sin(fail2), 0];  
point_C3 = [DELTA_R * cos(fail3), DELTA_R * sin(fail3), 0];  

% 计算点 D1, D2, D3 的坐标  
point_D1 = [(DELTA_R - DELTA_r + DELTA_L_ARM * cos(cita1) ) * cos(fail1), ...  
             (DELTA_R - DELTA_r + DELTA_L_ARM * cos(cita1) ) * sin(fail1), ...  
             -DELTA_L_ARM * sin(cita1)];  
         
point_D2 = [(DELTA_R - DELTA_r + DELTA_L_ARM * cos(cita2) ) * cos(fail2), ...  
             (DELTA_R - DELTA_r + DELTA_L_ARM * cos(cita2) ) * sin(fail2), ...  
             -DELTA_L_ARM * sin(cita2)];  
         
point_D3 = [(DELTA_R - DELTA_r + DELTA_L_ARM * cos(cita3) ) * cos(fail3), ...  
             (DELTA_R - DELTA_r + DELTA_L_ARM * cos(cita3) ) * sin(fail3), ...  
             -DELTA_L_ARM * sin(cita3)];  
         
% 计算点 OF 的坐标 (D中点)  
vector_OF = [(point_D2(1) + point_D3(1)) / 2, ...  
             (point_D2(2) + point_D3(2)) / 2, ...  
             (point_D2(3) + point_D3(3)) / 2];  

% 计算边长  
D2D3_a = norm(point_D3 - point_D2);  
D1D3_b = norm(point_D1 - point_D3);  
D1D2_c = norm(point_D1 - point_D2);  

% 计算海伦公式  
helen_p = (D2D3_a + D1D3_b + D1D2_c) / 2;  
area_s = sqrt(helen_p * (helen_p - D2D3_a) * (helen_p - D1D3_b) * (helen_p - D1D2_c));  
%外接圆边长
D2E = D2D3_a * D1D3_b * D1D2_c / (4 * area_s);  
EF = sqrt(D2E^2 - (D2D3_a / 2)^2);  

% 计算向量 D2D1, D2D3 和 D3D2  
vector_D2D1 = point_D1 - point_D2;  
vector_D2D3 = point_D3 - point_D2;  
vector_D3D2 = point_D2 - point_D3;  

% 计算法向量 n_FE  再看一下
n_FE = cross(vector_D2D1, vector_D2D3);  
n_FE = cross(vector_D3D2,n_FE);  
n_FE = n_FE / norm(n_FE); % 归一化  
%计算D2和D2D3的叉积
% cross_D2_D2D3 = cross(point_D2,vector_D2D3)
% cross_D2D3_D3 = cross(cross_D2_D2D3,point_D3)
% n_FE = cross(cross_D2_D2D3,cross_D2D3_D3)
%计算模
norm_D2 = norm(point_D2)
norm_D2D3 = norm(vector_D3D2)
norm_D3 = norm(point_D3)
% 计算向量 FE  
vector_FE = n_FE / (norm_D2 * norm_D2D3 * norm_D3)  

% 计算 EP  
EP = sqrt(DELTA_A_ARM^2 - D2E^2);  %这里再看一下
n_EP = cross(vector_D2D1, vector_D2D3);  
n_EP = n_EP / norm(n_EP); % 归一化  
vector_EP = n_EP * EP;  
 
% 计算最终向量 OP  
vector_OP = vector_OF + vector_FE + vector_EP;  

% 更新控制器的 TCP 坐标  
TCP_XYZ(1) = vector_OP(3); % z  
TCP_XYZ(2) = -vector_OP(2); % -y  
TCP_XYZ(3) = vector_OP(1); % x  
 angle = detal_test2(vector_OP)