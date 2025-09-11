%% dataPostproccess_createSubmap - 多波束数据后处理与子地图生成工具
%
% 功能描述：
%   对多波束采集数据进行后处理，生成子地图，并转换为SLAM所需格式。
%   包括数据加载、误差添加、子地图划分、坐标转换等功能
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.2
%   创建日期：241219
%   最后修改：250826
%
% 版本历史：
%   v1.2 (250826) - 新增INS误差检测功能
%       + 支持固定文件名的INS误差数据检测
%       + 添加条件性误差注入逻辑
%       + 优化日志输出和错误处理
%   v1.1 (250104) - 更新
%       + 集成惯导误差数据处理
%       + 优化子地图生成算法
%       + 改进文件组织结构
%   v1.0 (241219) - 首次发布
%       + 实现基础子地图生成
%       + 支持PCD格式转换
%       + 添加坐标系转换
%
% 输入文件：
%   - *_recoder.mat                  - 多波束记录数据
%   - *_Ins_path_simulated_data.mat  - 惯导轨迹数据（可选，不存在时跳过误差注入）
%
% 输出文件：
%   - *_sub_maps_data.mat           - 子地图数据
%   - /TXT_sub_maps/sub_map_*.txt   - TXT格式子地图
%   - /PCD_sub_maps/submap_*_frame.pcd - PCD格式子地图
%
% 主要功能：
%   1. 加载与预处理多波束数据
%   2. 添加仿真惯导误差
%   3. 生成规则子地图
%   4. 坐标系转换
%   5. 格式转换输出
%
% 注意事项：
%   1. 确保输入数据完整性
%   2. 子地图划分参数需按实际需求调整
%   3. 注意磁盘空间要求
%
% 调用示例：
%   % 直接运行脚本即可
%   dataPostproccess_createSubmap
%
% 依赖工具箱：
%   - Point Cloud Library
%
% 参见函数：
%   createSubmap, coordinateTransform, submap2PCD

%% 初始化
clc;
clear;
close all;

%% 配置文件加载
cfg = config();

% 获取当前脚本所在路径
current_script_path = fileparts(mfilename('fullpath'));
% 设置存储路径为当前脚本路径的上一级文件夹下的Data文件夹
data_path = fullfile(current_script_path, '..', 'Data');

% 如果目录不存在，则创建它
if ~exist(data_path, 'dir')
    mkdir(data_path);
end

% 添加当前目录及子目录到搜索路径 
addpath(genpath(fileparts(current_script_path)));

% 载入数据
recoder_file = '250826_recoder.mat';
load(fullfile(data_path, recoder_file));

ins_file = fullfile(data_path, '250826_Ins_path_simulated_data.mat');
if exist(ins_file, 'file')
    S = load(ins_file);
    if isfield(S,'ins_path_simulated') && isfield(S,'ins_simulated_error')
        recoder_with_ins_error = addNoiseToRecorder(recoder, S.ins_path_simulated, S.ins_simulated_error);
        fprintf('[INFO] dataPostproccess_createSubmap: 使用 250104_Ins_path_simulated_data.mat 应用误差。\n');
    else
        recoder_with_ins_error = recoder;
        fprintf('[WARN] dataPostproccess_createSubmap: INS 文件缺少关键字段，跳过噪声注入。\n');
    end
else
    recoder_with_ins_error = recoder;
    fprintf('[INFO] dataPostproccess_createSubmap: 未发现固定 INS 文件，使用原始 recoder。\n');
end
fprintf('Step 1 - 载入数据完成\n');
fprintf('Step 2 - 误差处理完成\n');

% 可视化点云
visualizeRecoderPointCloud(recoder_with_ins_error, 'cloud');

%% 子地图生成
% 从配置文件构建子地图参数
submap_params.heading_threshold = cfg.submap.heading_threshold;
submap_params.frames_per_submap = cfg.submap.frames_per_submap;
submap_params.window_size = cfg.submap.window_size;

[~,submap_data,~] = createSubmap(recoder,recoder_with_ins_error, submap_params);

% 输出子地图生成统计信息
num_submaps = length(submap_data);
fprintf('\n======================================\n');
fprintf('📍 子地图生成完成！\n');
fprintf('📊 生成子地图数量: %d 个\n', num_submaps);

% 输出详细统计信息
if num_submaps > 0
    submap_sizes = cellfun(@(x) size(x, 1), submap_data);
    fprintf('📈 子地图统计:\n');
    fprintf('   - 平均帧数: %.1f 帧/子地图\n', mean(submap_sizes));
    fprintf('   - 最大帧数: %d 帧\n', max(submap_sizes));
    fprintf('   - 最小帧数: %d 帧\n', min(submap_sizes));
    fprintf('   - 总帧数: %d 帧\n', sum(submap_sizes));
    
    % 显示每个子地图的帧数（如果子地图数量不太多的话）
    if num_submaps <= 20
        fprintf('📋 各子地图帧数: [');
        for i = 1:num_submaps
            fprintf('%d', submap_sizes(i));
            if i < num_submaps
                fprintf(', ');
            end
        end
        fprintf('] 帧\n');
    end
end
fprintf('======================================\n');
fprintf('Step 3.1 - 子地图划分完成\n');

% 保存子地图数据
save_date_time = datetime('now');
submap_filename = sprintf('%02d%02d%02d_sub_maps_data.mat', ...
                        mod(year(save_date_time),100), month(save_date_time), day(save_date_time));
save(fullfile(data_path, submap_filename), 'submap_data');
fprintf('Step 3.2 - 子地图数据保存完成: %s\n', submap_filename);

%% 子地图坐标系转换
% 创建保存目录
save_date_str = sprintf('%02d%02d%02d_sub_maps', ...
                        mod(year(save_date_time),100), month(save_date_time), day(save_date_time));
submap_root_dir = fullfile(data_path, save_date_str);
submap_txt_dir = fullfile(submap_root_dir, 'TXT_sub_maps');
submap_pcd_dir = fullfile(submap_root_dir, 'PCD_sub_maps');
% 创建目录
if ~exist(submap_txt_dir, 'dir') || ~exist(submap_pcd_dir, 'dir')
    try
        mkdir(fullfile(data_path));
        mkdir(submap_root_dir);
        mkdir(submap_txt_dir);
        mkdir(submap_pcd_dir);
    catch err
        error('目录创建失败：%s', err.message);
    end
end

% 坐标转换：全局坐标系转关键帧载体坐标系
key_frame_data = coordinateTransform(submap_data, submap_txt_dir);
fprintf('Step 4 - 坐标转换完成\n');

%% 生成PCD格式子地图
submap2PCD(submap_txt_dir, submap_pcd_dir)
fprintf('Step 5 - PCD格式转换完成\n');

% 输出最终完成信息
fprintf('\n✅ 数据后处理完成！子地图文件已保存至:\n');
fprintf('   📁 TXT格式: %s\n', submap_txt_dir);
fprintf('   📁 PCD格式: %s\n', submap_pcd_dir);
fprintf('   📊 共生成 %d 个子地图文件\n', num_submaps);
fprintf('   💾 子地图数据文件: %s\n\n', fullfile(data_path, submap_filename));

% load KEY_FRAME.mat;
% load path_ins.mat;
% update_pdc_files('D:\code\算法—整理版\工具\submap_regular\submap',...
%                 'D:\code\算法—整理版\工具\子地图与惯导固连\惯导路径子地图' , ...
%                 KEY_FRAME, ...
%                 path_ins)
% delete_nan('D:\code\算法—整理版\工具\子地图与惯导固连\惯导路径子地图', ...
%             'D:\code\算法—整理版\工具\子地图与惯导固连\惯导路径子地图去除空值')
