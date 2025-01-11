%% simulateMultibeam - 多波束声呐仿真采集主函数
%
% 功能描述：
%   执行多波束声呐仿真采集的主控制函数，包括地形数据预处理、
%   多波束扫描仿真和点云数据可视化
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.2
%   创建日期：240409
%   最后修改：240409
%
% 版本历史：
%   v1.2 (240409) - 新增功能
%       + 添加点云数据可视化功能
%       + 优化数据处理流程
%   v1.1 (240408) - 功能完善
%       + 实现基础多波束仿真功能
%
% 输入参数：
%   processedPath - [Nx3 double] 预处理后的AUV路径数据
%                   [x y heading]，heading为角度制
%   terrainData  - [struct] 地形数据结构体
%                   .X - 地形X坐标矩阵
%                   .Y - 地形Y坐标矩阵
%                   .Z - 地形深度矩阵
%
% 输出参数：
%   recoder - [Nx(3M+5) double] 多波束采集记录数据矩阵
%             第1列：帧数
%             第2-4列：AUV位姿 [x y heading]
%             第5列：AUV深度
%             第6列之后：多波束点云数据 [x1 y1 z1 ... xM yM zM]
%
% 注意事项：
%   1. 输入地形数据必须是规则网格
%   2. AUV路径必须在地形范围内
%   3. 建议预先检查内存是否充足
%
% 调用示例：
%   % 加载地形数据
%   load('terrain_data.mat');
%   % 加载预处理的路径
%   load('processed_path.mat');
%   % 执行仿真
%   recoder = simulateMultibeam(processed_path, terrain_data);
%
% 依赖函数：
%   - multibeam.m
%   - pcshow (MATLAB Point Cloud Toolbox)

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
