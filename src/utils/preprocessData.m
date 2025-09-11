%% preprocessData - AUV数据预处理与INS误差模拟工具
%
% 功能描述：
%   对AUV原始路径数据进行预处理，包括降采样、坐标变换，并模拟INS误差
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.4
%   创建日期：250104
%   最后修改：250429
%
% 版本历史：
%   v1.4 (250826) - 新增INS误差开关功能
%       + 添加 enable_ins_error 开关控制误差模拟
%       + 支持关闭误差时返回空数组语义
%       + 优化向后兼容性和错误处理
%   v1.3 (250104) - 新增INS误差模拟功能
%       + 添加INS误差生成模块
%       + 优化路径可视化
%   v1.2 (250103) - 优化降采样算法
%       + 改进数据降采样方法
%       + 添加数据验证功能
%   v1.1 (250102) - 实现基础数据预处理
%       + 实现基本数据处理功能
%       + 添加数据导出功能
%
% 输入参数：
%   pathData - [NxM] 原始AUV路径数据矩阵
%   params   - 参数结构体
%     .enable_ins_error   - [bool] INS误差模拟开关(可选，默认true)
%     .target_points      - [int] 降采样目标点数
%     .scale_factor      - [double] 坐标缩放系数
%     .error            - INS误差参数结构体（当enable_ins_error=true时必需）
%       .line_std       - [1x2] 直线段误差标准差 [x,y]
%       .turn_std       - [1x2] 转弯段误差标准差 [x,y]
%       .cumulative     - [1x2] 累积误差因子 [x,y]
%       .turn_factor    - [1x2] 转弯误差系数 [x,y]
%       .no_error_fraction - [double] 无误差路径比例 (0-1)
%       .window_size    - [int] 滑动窗口大小
%
% 输出参数：
%   processedPath - [Nx3] 处理后的理想AUV路径 [x,y,heading]
%   insPath      - [Nx3] 添加INS误差后的路径，关闭误差时为空数组[]
%   insError     - [Nx3] INS误差数据，关闭误差时为空数组[]
%
% 注意事项：
%   1. 数据格式：确保输入数据格式正确
%   2. 内存要求：大数据集可能需要较大内存
%   3. 处理时间：与数据量成正比
%   4. 误差开关：关闭误差时(enable_ins_error=false)返回空数组语义
%
% 调用示例：
%   % 定义参数结构体
%   params.target_points = 1000;
%   params.scale_factor = 10;
%   params.error = struct('line_std',[0.1,0.1],...);
%   
%   % 处理数据
%   [path, ins, err] = preprocessData(raw_data, params);
%
% 依赖工具箱：
%   - Statistics and Machine Learning Toolbox
%   - Signal Processing Toolbox
%
% 参见函数：
%   generateSimulatedInsPath, downsample, plot

function [processedPath, insPath, insError] = preprocessData(pathData, params, cfg)    
    original_following = pathData;

    %% 开关默认值（向后兼容）
    if ~isfield(params, 'enable_ins_error')
        params.enable_ins_error = true; % 保持历史默认行为
    end
    
    %% 数据降采样
    % 计算降采样率
    n = length(original_following);
    downsample_rate = ceil(n / params.target_points);
    
    % 降采样
    following_downsampled = downsample(original_following, downsample_rate);
    if size(following_downsampled,1) > params.target_points
        following_downsampled = following_downsampled(1:params.target_points, :);
    end
    
    fprintf(' - 降采样: %d -> %d 点\n', n, size(following_downsampled,1));
    
    % ===== 以下数据裁剪逻辑已被 config.m 统一管理，保留注释供参考 =====
    % 删除多余数据!!!(选用) - 原硬编码逻辑
    % NESP 700开始, noNESP 800开始
    % if size(following_downsampled,1) > 800
    %     following_downsampled = following_downsampled(801:end, :);  % 删除前 800 行
    % end
    % shortTest - 原硬编码逻辑
    % following_downsampled = following_downsampled(701:5000, :);      % shortTest,只用1101:5000行
    
    % 使用配置文件中的数据裁剪逻辑
    if cfg.preprocess.trim_start_enabled && size(following_downsampled,1) > cfg.preprocess.trim_start_index
        following_downsampled = following_downsampled(cfg.preprocess.trim_start_index:end, :);
        fprintf(' - 数据头部裁剪: 删除前 %d 行\n', cfg.preprocess.trim_start_index - 1);
    end
    
    % 短时测试模式
    if cfg.preprocess.short_test_enabled
        start_idx = cfg.preprocess.short_test_range(1);
        end_idx = cfg.preprocess.short_test_range(2);
        if size(following_downsampled,1) >= end_idx
            following_downsampled = following_downsampled(start_idx:end_idx, :);
            fprintf(' - 短时测试模式: 使用数据行 %d 到 %d\n', start_idx, end_idx);
        else
            fprintf(' - 警告: 数据不足以进行短时测试，跳过短时测试设置\n');
        end
    end
    
    %% 计算艏向角 - 并行优化版本
    % n_points = size(following_downsampled, 1);
    % 
    % % 计算相邻点之间的差值
    % dx = diff(following_downsampled(:,1));
    % dy = diff(following_downsampled(:,2));
    % 
    % % 向量化计算艏向角
    % headings = [0; atan2(dy, dx)];
    % 
    % % 第一个点的艏向角采用第二个点的艏向角
    % headings(1) = headings(2);
    % 
    % fprintf('Step 3 - 并行计算艏向角完成\n');
    
    %% 处理数据格式并保存
    % 合并路径和艏向角数据
    following_downsampled_temp = following_downsampled;
    following_downsampled_temp(:, 1:2) = following_downsampled(:,1:2)/params.scale_factor; % 缩放坐标
    % 保持角度原单位（如后续需弧度再统一处理）
    % following_downsampled_temp(:, 3) = deg2rad(following_downsampled(:,3));
    following_downsampled_temp(:, 3) = following_downsampled(:,3); % 保持度
    processed_path = following_downsampled_temp;
    
    % 获取当前路径
    currentPath = pwd;
    dataPath = fullfile(currentPath,'Data');
    % 生成带时间戳的文件名 (使用datetime直接格式化)
    current_date = datetime('now');
    filename = sprintf('%02d%02d%02d_Processed_path_data.mat', ...
        mod(year(current_date),100), ...
        month(current_date), ...
        day(current_date));
    % 保存处理后的数据
    save(fullfile(dataPath,filename), 'processed_path');
    
    %% 绘制路径图进行验证
    figure;
    plot(processed_path(:,1), processed_path(:,2), 'b-');
    hold on;
    % 每隔100个点绘制一个箭头表示艏向
    arrow_interval = 100;
    
    quiver(processed_path(1:arrow_interval:end,1), ...
           processed_path(1:arrow_interval:end,2), ...
           cos(deg2rad(processed_path(1:arrow_interval:end,3))), ...
           sin(deg2rad(processed_path(1:arrow_interval:end,3))), ...
           0.5, 'r');
    title('AUV路径与艏向示意图');
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on;
    axis equal;
    
    %% ============================== INS误差模拟（可选） =========================================
    processedPath = following_downsampled_temp;
    
    if params.enable_ins_error
        % === INS ERROR SIMULATION BEGIN ===
        gps_path = processed_path; % Nx3 矩阵 [x, y, heading]

        % 设置误差参数
        line_error_std_x = params.error.line_std(1);
        line_error_std_y = params.error.line_std(2);
        turn_error_std_x = params.error.turn_std(1);
        turn_error_std_y = params.error.turn_std(2);
        cumulative_error_factor_x = params.error.cumulative(1);
        cumulative_error_factor_y = params.error.cumulative(2);
        turn_error_factor_x = params.error.turn_factor(1);
        turn_error_factor_y = params.error.turn_factor(2);
        no_error_fraction = params.error.no_error_fraction;
        window_size = params.error.window_size;

        % 生成仿真路径
        [ins_path_simulated, ins_simulated_error] = generateSimulatedInsPath(gps_path, ...
            line_error_std_x, line_error_std_y, ...
            turn_error_std_x, turn_error_std_y, ...
            cumulative_error_factor_x, cumulative_error_factor_y, ...
            turn_error_factor_x, turn_error_factor_y, ...
            no_error_fraction, window_size);

        % 误差统计
        position_error = sqrt(ins_simulated_error(:, 1).^2 + ins_simulated_error(:, 2).^2);
        path_diff = diff(gps_path(:, 1:2));
        path_segment_lengths = sqrt(sum(path_diff.^2, 2));
        total_path_length = sum(path_segment_lengths);
        final_position_error = position_error(end);
        error_rate = (final_position_error / total_path_length) * 1000; % 千分之

        fprintf('[INS] 路径总长度: %.2f m | 最终位置误差: %.2f m | 误差率: %.2f‰\n', ...
            total_path_length, final_position_error, error_rate);

        % 可视化
        figure('Name','INS误差演化');
        subplot(3, 1, 1);
        plot(ins_simulated_error(:, 1), 'r-', 'LineWidth', 1.1); hold on;
        plot(ins_simulated_error(:, 2), 'b-', 'LineWidth', 1.1);
        legend('X方向误差', 'Y方向误差'); grid on; xlabel('步'); ylabel('误差(m)');
        title('X/Y方向误差');
        subplot(3,1,2); plot(position_error,'k-','LineWidth',1.1); grid on; xlabel('步'); ylabel('误差(m)'); title('位置误差模');
        subplot(3,1,3); scatter(ins_simulated_error(:,1), ins_simulated_error(:,2), 8, (1:size(ins_simulated_error,1)),'filled');
        xlabel('X误差'); ylabel('Y误差'); title('误差散点'); grid on; colorbar;

        figure('Name','路径对比');
        plot(gps_path(:,1), gps_path(:,2),'b-'); hold on; plot(ins_path_simulated(:,1), ins_path_simulated(:,2),'r-');
        legend('理想(GPS)','仿真(INS)'); axis equal; grid on; title('GPS vs INS 路径'); xlabel('X'); ylabel('Y');

        % 保存 INS 路径
        current_date = datetime('now');
        filename = sprintf('%02d%02d%02d_Ins_path_simulated_data.mat', ...
            mod(year(current_date),100), month(current_date), day(current_date));
        save(fullfile(dataPath,filename), 'ins_path_simulated','ins_simulated_error');

        insPath = ins_path_simulated;
        insError = ins_simulated_error;
        % === INS ERROR SIMULATION END ===
    else
        insPath = [];
        insError = [];
        fprintf('[INFO] preprocessData: INS误差模拟未启用，跳过误差段。\n');
    end
end
