% main - 多波束声呐模拟地形采集工具链主函数
% 日期：250104
% 作者：Chihong（游子昂）
% 版本：v1.1 完整工具链流程实现
%
% 功能说明：
%   1. 数据预处理：路径降采样与INS误差模拟
%   2. 多波束仿真：模拟声呐采集过程
%   3. 数据后处理：子地图生成与存储
%
% 参数配置：
%   1. 路径处理参数
%       - 降采样目标点数
%       - 坐标缩放系数
%   2. INS误差参数
%       - 直线段与转弯段误差
%       - 累积误差与转弯系数
%
% 输入文件：
%   - 地形数据：MapPoint_900_900.mat
%   - 路径数据：PathFollowing_1.mat

function main()
    % 清理工作环境
    clc; close all;
    
    % 获取当前工作目录
    currentPath = pwd;
    addpath(genpath(currentPath));
    
    try
        %% 0. 参数配置及数据加载
        % 路径处理参数
        params.target_points = 70000;      % 降采样目标点数
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
