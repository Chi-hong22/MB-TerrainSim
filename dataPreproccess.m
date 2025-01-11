%% dataPreproccess - AUV路径数据预处理与惯导误差仿真工具
%
% 功能描述：
%   处理AUV路径数据并生成仿真惯导误差，包括数据降采样、坐标变换、
%   惯导误差模拟等功能
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.1
%   创建日期：241230
%   最后修改：250104
%
% 版本历史：
%   v1.1 (250104) - 更新
%       + 优化惯导误差生成算法
%       + 添加详细误差分析功能
%       + 改进数据可视化效果
%   v1.0 (241230) - 首次发布
%       + 实现基础数据预处理功能
%       + 支持路径降采样
%       + 添加基本误差模拟
%
% 输入文件：
%   - *_PathFollowing_*.mat     - 原始路径数据
%   - *_Processed_path_data.mat - GPS参考路径数据（可选）
%
% 输出文件：
%   - *_Processed_path_data.mat     - 处理后的路径数据
%   - *_Ins_path_simulated_data.mat - 带误差的仿真惯导路径
%
% 主要参数：
%   line_error_std_*     - [double] 直线段误差标准差
%   turn_error_std_*     - [double] 转弯段误差标准差
%   cumulative_error_factor_* - [double] 累积误差因子
%   no_error_fraction    - [double] 无误差路径比例
%
% 注意事项：
%   1. 路径数据需包含位置和艏向信息
%   2. 误差参数需根据实际情况调整
%   3. 建议对生成的误差进行验证
%
% 调用示例：
%   % 直接运行脚本即可
%   dataPreproccess
%
% 依赖工具箱：
%   - Statistics and Machine Learning Toolbox
%
% 参见函数：
%   generateSimulatedInsPath, downsample

%% 对通过动力学模型跑出来的AUV路径进行前处理

%% ==============================NESP路径处理=========================================%
% 之后采用多波束模拟算法采集点云数据，牛师兄多波束模拟脚本 `main_multibeamSimulink.m`
clc;
clear;
close all;

%% 载入数据
% load 241216_MapPoint_900_900.mat % NESP地形数据
load 250103_PathFollowing_1.mat
original_following = PathFollowing; 

fprintf('Step 1 - 载入数据\n');

%% 数据降采样
% 计算降采样率
n = length(original_following);
target_points = 70000;
downsample_rate = ceil(n / target_points);

% 降采样
following_downsampled = downsample(original_following, downsample_rate);
if size(following_downsampled,1) > target_points
    following_downsampled = following_downsampled(1:target_points, :);
end

fprintf('Step 2 - 降采样: %d -> %d 点\n', n, size(following_downsampled,1));

% 删除多余数据
following_downsampled = following_downsampled(501:end, :);      % 删除前 500 行

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
following_downsampled_temp(:, 1:2) = following_downsampled(:,1:2)/6; % 缩放坐标
following_downsampled_temp(:, 3) = deg2rad(following_downsampled(:,3));% 添加角度转弧度的转换步骤
following_downsampled_temp(:, 3) = following_downsampled(:,3);
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
fprintf('Step 4 - 数据保存完成: %s\n', filename);

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

%% ==============================真实实验数据噪声提取=========================================%
% 将误差模式应用到处理后的路径

% 加载数据
% load('Data/241230_Processed_path_data.mat'); % 本地加载GPS路径

gps_path = processed_path; % Nx3 矩阵 [x, y, heading]

% 设置误差参数
line_error_std_x = 0.03; % 直线部分 X 方向误差标准差
line_error_std_y = 0.05; % 直线部分 Y 方向误差标准差
turn_error_std_x = 0.0005; % 转弯部分 X 方向误差标准差
turn_error_std_y = 0.02; % 转弯部分 Y 方向误差标准差
cumulative_error_factor_x = 0.019; % X 方向累积误差因子
cumulative_error_factor_y = 0.0025; % Y 方向累积误差因子
turn_error_factor_x = 0.01; % 转弯误差 X 方向系数
turn_error_factor_y = 0.01; % 转弯误差 Y 方向系数
no_error_fraction = 0.03; % 前 3% 的路径不增加误差
window_size = 40; % 滑动窗口大小

% 调用函数生成仿真路径
[ins_path_simulated, ins_simulated_error] = generateSimulatedInsPath(gps_path, ...
                                                                line_error_std_x, line_error_std_y, ...
                                                                turn_error_std_x, turn_error_std_y, ...
                                                                cumulative_error_factor_x, cumulative_error_factor_y, ...
                                                                turn_error_factor_x, turn_error_factor_y, ...
                                                                no_error_fraction, window_size);

% 计算位置误差的欧几里得距离
position_error = sqrt(ins_simulated_error(:, 1).^2 + ins_simulated_error(:, 2).^2);

% 计算路径总长度
path_diff = diff(gps_path(:, 1:2)); % 逐点差分计算路径段
path_segment_lengths = sqrt(sum(path_diff.^2, 2)); % 每段路径长度
total_path_length = sum(path_segment_lengths); % 总路径长度

% 计算最终位置误差
final_position_error = position_error(end); % 最后一点位置误差

% 计算误差率（单位：千分之）
error_rate = (final_position_error / total_path_length) * 1000; % 千分之

% 打印误差率
fprintf('路径总长度: %.2f m\n', total_path_length);
fprintf('最终位置误差: %.2f m\n', final_position_error);
fprintf('误差率: %.2f‰\n', error_rate); % 输出为千分之

% 绘制误差随时间变化
figure;
subplot(3, 1, 1);
plot(1:size(ins_simulated_error, 1), ins_simulated_error(:, 1), 'r-', 'LineWidth', 1.2); hold on;
plot(1:size(ins_simulated_error, 1), ins_simulated_error(:, 2), 'b-', 'LineWidth', 1.2);
legend('X方向误差', 'Y方向误差');
xlabel('时间步长');
ylabel('误差 (m)');
title('X和Y方向误差随时间变化');
grid on;

% 绘制位置误差随时间变化
subplot(3, 1, 2);
plot(1:size(position_error, 1), position_error, 'k-', 'LineWidth', 1.2);
xlabel('时间步长');
ylabel('误差 (m)');
title('位置误差随时间变化');
grid on;

% 绘制误差分布 (X-Y平面)
subplot(3, 1, 3);
scatter(ins_simulated_error(:, 1), ins_simulated_error(:, 2), 10, 1:size(ins_simulated_error, 1), 'filled');
xlabel('X方向误差 (m)');
ylabel('Y方向误差 (m)');
title('X-Y平面误差分布');
colorbar;
grid on;

% 绘制路径对比
figure;
plot(gps_path(:, 1), gps_path(:, 2), 'b-', 'LineWidth', 1); hold on;
plot(ins_path_simulated(:, 1), ins_path_simulated(:, 2), 'r-', 'LineWidth', 1);
legend('原始路径 (GPS)', '仿真路径 (INS)');
xlabel('X 坐标 (m)');
ylabel('Y 坐标 (m)');
title('GPS路径与仿真INS路径对比');
grid on;

%% 保存对应噪声INS路径
% 获取当前路径
currentPath = pwd;
dataPath = fullfile(currentPath,'Data');
% 生成带时间戳的文件名 (使用datetime直接格式化)
current_date = datetime('now');
filename = sprintf('%02d%02d%02d_Ins_path_simulated_data.mat', ...
    mod(year(current_date),100), ...
    month(current_date), ...
    day(current_date));
% 保存处理后的数据
save(fullfile(dataPath,filename), 'ins_path_simulated','ins_simulated_error');