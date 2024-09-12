% 定义常量 (请根据实际值设置这些常量)  
DELTA_R = 0.1;       % 半径或类似参数  
DELTA_L_ARM = 0.06;   % 臂的长度  
DELTA_r = 0.04;       % 另一个半径或偏差  
DELTA_A_ARM = 0.12;   % 可能代表臂的某个补偿值  
PI = 3.141592653589793; % 圆周率  

fail1 = 0;  
fail2 = 2 * PI / 3;  
fail3 = 4 * PI / 3;  

% 定义 cita1, cita2, cita3 的范围
cita_range = linspace(0, PI/2, 15); % 从 0 到 π/2 分成 15 个点  

% 初始化存储 TCP_XYZ 的工作空间
workspace_TCP_XYZ = []; 

% 遍历 cita1, cita2, cita3 的不同值
for i = 1:length(cita_range)
    for j = 1:length(cita_range)
        for k = 1:length(cita_range)
            % 获取当前的 cita1, cita2, cita3
            cita1 = cita_range(i);  
            cita2 = cita_range(j);  
            cita3 = cita_range(k);  

            % 计算点 C1, C2, C3 的坐标  
            point_C1 = [DELTA_R * cos(fail1), DELTA_R * sin(fail1), 0];  
            point_C2 = [DELTA_R * cos(fail2), DELTA_R * sin(fail2), 0];  
            point_C3 = [DELTA_R * cos(fail3), DELTA_R * sin(fail3), 0];  

            % 计算点 D1, D2, D3 的坐标  
            point_D1 = [(DELTA_R + DELTA_L_ARM * cos(cita1) - DELTA_r) * cos(fail1), ...  
                         (DELTA_R + DELTA_L_ARM * cos(cita1) - DELTA_r) * sin(fail1), ...  
                         -DELTA_L_ARM * sin(cita1)];  

            point_D2 = [(DELTA_R + DELTA_L_ARM * cos(cita2) - DELTA_r) * cos(fail2), ...  
                         (DELTA_R + DELTA_L_ARM * cos(cita2) - DELTA_r) * sin(fail2), ...  
                         -DELTA_L_ARM * sin(cita2)];  

            point_D3 = [(DELTA_R + DELTA_L_ARM * cos(cita3) - DELTA_r) * cos(fail3), ...  
                         (DELTA_R + DELTA_L_ARM * cos(cita3) - DELTA_r) * sin(fail3), ...  
                         -DELTA_L_ARM * sin(cita3)];  

            % 计算点 OF 的坐标 (中点)  
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
            D2E = D2D3_a * D1D3_b * D1D2_c / (4 * area_s);  
            EF = sqrt(D2E^2 - (D2D3_a / 2)^2);  

            % 计算向量 D2D1, D2D3 和 D3D2  
            vector_D2D1 = point_D1 - point_D2;  
            vector_D2D3 = point_D3 - point_D2;  
            vector_D3D2 = point_D2 - point_D3;  

            % 计算法向量 n_FE  
            n_FE = cross(vector_D2D1, vector_D2D3);  
            n_FE = cross(n_FE, vector_D3D2);  
            n_FE = n_FE / norm(n_FE); % 归一化  

            % 计算向量 FE  
            vector_FE = n_FE * EF;  

            % 计算 EP  
            EP = sqrt(DELTA_A_ARM^2 - D2E^2);  
            n_EP = cross(vector_D2D1, vector_D2D3);  
            n_EP = n_EP / norm(n_EP); % 归一化  
            vector_EP = n_EP * EP;  

            % 计算最终向量 OP  
            vector_OP = vector_OF + vector_FE + vector_EP;  

            % 更新控制器的 TCP 坐标  
            TCP_XYZ(1) = vector_OP(3); % z  
            TCP_XYZ(2) = -vector_OP(2); % -y  
            TCP_XYZ(3) = vector_OP(1); % x  

            % 存储到工作空间数组中
            workspace_TCP_XYZ = [workspace_TCP_XYZ; TCP_XYZ];
        end
    end
end

% 去除复数部分（如果有）
workspace_TCP_XYZ = real(workspace_TCP_XYZ);

% 绘制 TCP_XYZ 的工作空间
figure;
scatter3(workspace_TCP_XYZ(:,1), workspace_TCP_XYZ(:,2), workspace_TCP_XYZ(:,3), 1, '.');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('TCP XYZ Workspace for cita1, cita2, cita3 in range 0 to π/2');
grid on;
axis equal;

% 使用 alphaShape 绘制边界曲线
shp = alphaShape(workspace_TCP_XYZ, 1.0); % alpha 值根据需要调整
hold on;
plot(shp, 'FaceColor', 'cyan', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
