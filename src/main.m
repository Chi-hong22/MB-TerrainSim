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
%   当前版本：v1.2
%   创建日期：250104
%   最后修改：250429
%
% 版本历史：
%   v1.0 (250103) - 初始版本
%       + 基础功能实现
%   v1.1 (250104) - 功能完善
%       + 添加完整工具链处理流程
%       + 优化参数配置结构
%       + 增强错误处理机制
%   v1.2 (250826) - INS误差开关功能
%       + 新增 enable_ins_error 开关控制INS误差模拟
%       + 支持理想路径和含误差路径双模式运行
%       + 优化路径选择逻辑和错误处理
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
%   1. 误差控制参数
%       enable_ins_error - INS误差模拟开关(true/false, 默认true)
%   2. 路径处理参数
%       target_points    - 降采样后的目标点数(默认70000)
%       scale_factor    - 坐标系缩放系数(默认6)
%   3. 惯导误差参数（当enable_ins_error=true时生效）
%       line_std        - 直线段随机误差标准差[x,y]
%       turn_std        - 转弯段随机误差标准差[x,y]
%       cumulative      - 累积误差系数[x,y]
%       turn_factor     - 转弯误差系数[x,y]
%       window_size     - 艏向角平滑窗口大小
%
% 输入数据：
%   1. 地形数据文件：MapPoint_900_900.mat
%       - 包含三个矩阵变量：X、Y、Z
%       - 实际尺寸：900×900
%       - 扩展尺寸：1100×1100 double (边缘各扩展100个采样点作为缓冲区，
%         用于确保多波束模拟过程中的边界安全)
%       - X、Y：表示采样点的平面坐标
%       - Z：表示对应点的地形高程值
%
%   2. 路径数据文件：PathFollowing_1.mat
%       - 包含变量：PathFollowing
%       - 数据结构：n×3 double矩阵
%       - 列说明：[x, y, theta]
%           x: AUV在X轴位置
%           y: AUV在Y轴位置
%           theta: AUV艏向角(角度制)
%       - n为路径采样点数量
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

    % 获取当前脚本所在路径
    current_script_path = fileparts(mfilename('fullpath'));
    % 设置数据路径为当前脚本路径的上一级文件夹下的Data文件夹
    data_path = fullfile(current_script_path, '..', 'Data');
    
    % 如果目录不存在，则创建它
    if ~exist(data_path, 'dir')
        mkdir(data_path);
    end
    
    % 添加当前目录及子目录到搜索路径 
    addpath(genpath(fileparts(current_script_path)));
    
    try
        %% 0. 参数配置及数据加载
    % 路径处理参数
        params.target_points = 70000;      % 降采样目标点数70000
        params.scale_factor = 6;           % 坐标缩放系数
        % INS误差开关（新增）
        params.enable_ins_error = false;    % 设为 false 可关闭 INS 误差模拟
        
        % 误差模拟参数
        params.error.line_std = [0.03, 0.05];          % 直线段误差标准差 [x, y]
        params.error.turn_std = [0.0005, 0.02];        % 转弯段误差标准差 [x, y]
        params.error.cumulative = [0.019, 0.0025];     % 累积误差因子 [x, y]
        params.error.turn_factor = [0.01, 0.01];       % 转弯误差系数 [x, y]
        params.error.no_error_fraction = 0.03;         % 无误差路径比例
        params.error.window_size = 40;                 % 滑动窗口大小
        
        fprintf('开始加载数据...\n');
        % 加载地形数据
        terrainData = load(fullfile(data_path, 'MapPoint_900_900.mat'));        % 地形数据
        
        % 加载AUV路径数据
        pathFile = load(fullfile(data_path, 'NESP_PathFollowing.mat'));              % 原始路径
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
        postprocessData(recoder, insPath, insError,data_path);
        fprintf('后处理与子图生成完成\n\n');
        
        fprintf('整个处理流程已完成！\n');
    catch ME
        fprintf('错误: %s\n', ME.message);
        rethrow(ME);
    end
end
