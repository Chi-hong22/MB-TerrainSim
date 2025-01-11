%% main - 多波束声呐地形采集仿真系统
%
% 功能描述：
%   实现多波束声呐系统的完整仿真链，包括路径规划、惯导误差模拟、
%   多波束采集仿真以及后处理等全流程功能。
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.1
%   创建日期：250104
%   最后修改：250104
%
% 版本历史：
%   v1.0 (250103) - 初始版本
%       + 基础功能实现
%   v1.1 (250104) - 功能完善
%       + 添加完整工具链处理流程
%       + 优化参数配置结构
%       + 增强错误处理机制
%
% 处理流程：
%   1. 数据预处理
%       - 路径数据降采样与平滑
%       - 惯导误差仿真与注入
%   2. 多波束仿真
%       - 声呐波束建模
%       - 地形交汇计算
%       - 数据记录与存储
%   3. 后处理
%       - 子地图分割与生成
%       - 数据格式化与导出
%
% 配置参数：
%   1. 路径处理参数
%       target_points    - 降采样后的目标点数(默认70000)
%       scale_factor    - 坐标系缩放系数(默认6)
%   2. 惯导误差参数
%       line_std        - 直线段随机误差标准差[x,y]
%       turn_std        - 转弯段随机误差标准差[x,y]
%       cumulative      - 累积误差系数[x,y]
%       turn_factor     - 转弯误差系数[x,y]
%       window_size     - 艏向角平滑窗口大小
%
% 输入数据：
%   地形数据：MapPoint_900_900.mat
%       - 包含测区地形高程信息
%       - 规格：900×900网格
%   路径数据：PathFollowing_1.mat
%       - 包含AUV航行轨迹信息
%       - 格式：[x, y, heading, depth]
%
% 注意事项：
%   1. 运行前确保数据文件存在且格式正确
%   2. 参数配置需根据实际场景调整
%   3. 建议预留充足内存空间（>4GB）
%
% 调用示例：
%   main()
%
% 依赖工具箱：
%   - Parallel Computing Toolbox (可选)
%   - Statistics and Machine Learning Toolbox
%
% 参见函数：
%   preprocessData, simulateMultibeam, postprocessData

function main()
    % 清理工作环境
    clc; close all;
    
    % 获取当前工作目录
    currentPath = pwd;
    addpath(genpath(currentPath));
    
    try
        %% 0. 参数配置及数据加载
        % 路径处理参数
        params.target_points = 70000;      % 降采样目标点数70000
        params.scale_factor = 6;           % 坐标缩放系数
        
        % 误差模拟参数
        params.error.line_std = [0.03, 0.05];          % 直线段误差标准差 [x, y]
        params.error.turn_std = [0.0005, 0.02];        % 转弯段误差标准差 [x, y]
        params.error.cumulative = [0.019, 0.0025];     % 累积误差因子 [x, y]
        params.error.turn_factor = [0.01, 0.01];       % 转弯误差系数 [x, y]
        params.error.no_error_fraction = 0.03;         % 无误差路径比例
        params.error.window_size = 40;                 % 滑动窗口大小
        
        fprintf('开始加载数据...\n');
        % 加载地形数据
        terrainData = load('Data/241216_MapPoint_900_900.mat');        % 地形数据
        
        % 加载AUV路径数据
        pathFile = load('Data/250103_PathFollowing_1.mat');              % 原始路径
        pathData = pathFile.PathFollowing;
        fprintf('数据加载完成\n\n');
        
        %% 1. 数据预处理
        fprintf('开始数据预处理...\n');
        [processedPath, insPath, insError] = preprocessData(pathData, params);
        fprintf('数据预处理完成\n\n');
        
        %% 2. 多波束仿真采集
        fprintf('开始多波束仿真采集...\n');
        recoder = simulateMultibeam(processedPath, terrainData);
        fprintf('多波束仿真采集完成\n\n');
        
        %% 3. 后处理与子图生成
        fprintf('开始后处理与子图生成...\n');
        postprocessData(recoder, insPath, insError);
        fprintf('后处理与子图生成完成\n\n');
        
        fprintf('整个处理流程已完成！\n');
    catch ME
        fprintf('错误: %s\n', ME.message);
        rethrow(ME);
    end
end
