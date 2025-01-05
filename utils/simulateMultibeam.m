% simulateMultibeam - 多波束声呐仿真采集函数
% 日期：250104
% 作者：Chihong（游子昂）
% 版本：v1.2 新增点云数据可视化功能
% 历史版本：v1.1 实现基础多波束仿真功能
%
% 输入参数:
%   processedPath - 预处理后的AUV路径数据
%       第1-2列：AUV的xy坐标
%       第3列：艏向角（角度制）
%   terrainData - 地形数据结构体
%       .X - 地形X坐标矩阵
%       .Y - 地形Y坐标矩阵
%       .Z - 地形深度矩阵
%
% 输出参数:
%   recoder - 多波束采集记录数据矩阵
%       第1列：帧数
%       第2-3列：AUV的xy坐标
%       第4列：艏向角
%       第5列：标识符
%       第6列及之后：多波束点云数据

function recoder = simulateMultibeam(processedPath, terrainData)
    %% 多波束仿真采集函数
    % 输入:
    %   processedPath - 预处理后的AUV路径
    %   terrainData - 地形数据结构体，包含 X, Y, Z
    % 输出:
    %   recoder - 多波束采集记录数据
    
    % 提取并放缩地形数据
    X = 10 * terrainData.X;
    Y = 10 * terrainData.Y;
    Z = terrainData.Z;
    
    % 提取AUV路径数据
    auv_x = processedPath(:, 1);
    auv_y = processedPath(:, 2);
    auv_heading = processedPath(:, 3);
    
    % 设置多波束参数
    SONAR_DEPTH = 0;
    SONAR_RANGE = 100;
    SONAR_ANGLE = 60;
    SONAR_BEAM_NUM = 256;
    TERRAIN_OFFSET = -25;
    
    % 执行多波束测深仿真
    auv_poses = [auv_x, auv_y, auv_heading];
    recoder = multibeam(X, Y, Z-TERRAIN_OFFSET, auv_poses, ...
        SONAR_DEPTH, SONAR_RANGE, SONAR_ANGLE, SONAR_BEAM_NUM);

    %% 提取点云数据 (跳过前5列姿态信息)
    point_cloud = reshape(recoder(:,6:end)', 3, [])';
    % 点云数据可视化
    figure;
    pcshow(point_cloud);
    title('多波束声呐采集点云');
    xlabel('X方向 (m)');
    ylabel('Y方向 (m)');
    zlabel('深度 (m)');
end
