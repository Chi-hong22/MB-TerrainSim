%% postprocessData - 多波束数据后处理工具
%
% 功能描述：
%   对多波束声呐采集的原始数据进行后处理，包括INS误差添加、
%   点云可视化和子地图生成，最终输出TXT和PCD格式的数据文件。
%   本函数作为多波束地形采集系统的后处理环节，将原始采集数据
%   分割成多个子地图，并实现不同坐标系之间的转换，最终生成
%   便于SLAM算法使用的标准格式点云数据。
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.3
%   创建日期：250104
%   最后修改：250429
%
% 版本历史：
%   v1.3 (250826) - 新增INS误差开关支持
%       + 支持检测空 INS 路径并跳过噪声注入
%       + 优化兼容性处理和错误日志
%       + 改进空数组语义处理
%   v1.2 (250104) - 功能增强
%       + 新增PCD格式输出支持
%       + 优化子地图生成算法
%   v1.1 (250103) - 完善功能
%       + 实现基础后处理功能
%       + 添加数据可视化
%
% 输入参数：
%   recoder   - [Nx(3M+5) double] 原始多波束采集记录数据，包含AUV位姿和测深点
%   insPath   - [Nx3 double] INS路径数据，空数组[]表示未启用INS误差
%   insError  - [Nx3 double] INS误差数据，空数组[]表示未启用INS误差
%   submap_root_dir - [string] 子地图根目录路径，用于存储生成的子地图数据
%
% 输出参数：
%   无直接返回值，在指定目录下生成处理结果：
%   1. TXT格式子地图 - 用于基本点云分析和可视化
%   2. PCD格式子地图 - 用于SLAM算法和进一步处理
%   3. 可视化结果图像 - 用于直观评估数据质量
%
% 处理流程：
%   1. 向原始记录添加INS误差模拟实际导航偏差
%   2. 可视化带误差的点云数据
%   3. 将连续数据分割成多个子地图
%   4. 进行坐标系转换（全局坐标→局部坐标）
%   5. 生成标准格式点云文件
%
% 注意事项：
%   1. 确保输入数据维度匹配
%   2. 需要足够的磁盘空间存储结果
%   3. 处理大量数据时注意内存使用
%   4. 空INS参数会跳过噪声注入，直接使用原始记录
%
% 调用示例：
%   % 指定数据目录为项目根目录下的Data文件夹
%   data_path = fullfile(fileparts(fileparts(mfilename('fullpath'))), '..', 'Data');
%   postprocessData(recoder, ins_path, ins_error, data_path);
%
% 依赖函数：
%   - addNoiseToRecorder
%   - createSubmap
%   - visualizeRecoderPointCloud
%   - coordinateTransform
%   - submap2PCD

function postprocessData(recoder, insPath, insError, submap_root_dir, cfg)
    % 兼容空 INS（未启用误差模拟）
    if nargin < 2 || isempty(insPath)
        hasIns = false;
    else
        hasIns = ~isempty(insPath);
    end
    
    if hasIns
        recoder_with_ins_error = addNoiseToRecorder(recoder, insPath, insError);
    else
        recoder_with_ins_error = recoder;
        fprintf('[INFO] postprocessData: 未提供 INS 路径，跳过噪声注入。\n');
    end

    % 可视化点云（理想或带误差）
    visualizeRecoderPointCloud(recoder_with_ins_error, 'cloud');

    % 从配置文件构建子地图参数
    submap_params.heading_threshold = cfg.submap.heading_threshold;
    submap_params.frames_per_submap = cfg.submap.frames_per_submap;
    submap_params.window_size = cfg.submap.window_size;

    % 生成子地图（使用原始与处理后组合逻辑）
    [~,submap_data,~] = createSubmap(recoder, recoder_with_ins_error, submap_params);
    
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
    fprintf('======================================\n\n');
    
    % % 可视化所有子地图点云
    % figure('Name', '所有子地图点云可视化', 'NumberTitle', 'off');
    % hold on;
    % for i = 1:length(submap_data)
    %     current_submap = submap_data{i};
    %     visualizeRecoderPointCloud(current_submap, 'cloud');
    % end
    % title('所有子地图点云数据可视化');
    
    % 创建保存目录
    save_date_time = datetime('now');
    save_date_str = sprintf('%02d%02d%02d_sub_maps', ...
        mod(year(save_date_time),100), month(save_date_time), day(save_date_time));
    
    current_path = pwd;
    submap_root_dir = fullfile(submap_root_dir, save_date_str);
    submap_txt_dir = fullfile(submap_root_dir, 'TXT_sub_maps');
    submap_pcd_dir = fullfile(submap_root_dir, 'PCD_sub_maps');
    
    % 创建目录结构
    mkdir(fullfile(current_path, 'Data'));
    mkdir(submap_root_dir);
    mkdir(submap_txt_dir);
    mkdir(submap_pcd_dir);
    
    % 坐标转换和PCD生成
    coordinateTransform(submap_data, submap_txt_dir);
    
    submap2PCD(submap_txt_dir, submap_pcd_dir);

    % 可选: 全局坐标聚合可视化
    try
        if isfield(cfg, 'visualization') && isfield(cfg.visualization, 'enable_global_view') && cfg.visualization.enable_global_view
            fprintf('[Viz] 开始加载PCD/PDC并进行全局可视化...\n');
            % 加载并变换到全局坐标
            measurements = loadAllSubmaps(submap_pcd_dir, 'Verbose', true, 'TransformToGlobal', true, 'UseParallel', true);
            if ~isempty(measurements)
                visualizeSubmaps(measurements, 'ColorBy', 'z', 'SampleRate', 1.0, 'Title', '子地图聚合(全局坐标)');
            else
                warning('[Viz] 未加载到有效子地图，跳过可视化');
            end
        end
    catch vizErr
        warning(vizErr.identifier, '%s', vizErr.message);
    end

    % 输出最终完成信息
    fprintf('✅ 后处理完成！子地图文件已保存至:\n');
    fprintf('   📁 TXT格式: %s\n', submap_txt_dir);
    fprintf('   📁 PCD格式: %s\n', submap_pcd_dir);
    fprintf('   📊 共生成 %d 个子地图文件\n\n', num_submaps);
end
