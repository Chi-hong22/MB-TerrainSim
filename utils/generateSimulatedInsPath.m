%% generateSimulatedInsPath - 惯导轨迹仿真生成工具
%
% 功能描述：
%   基于输入的GPS轨迹数据，模拟惯性导航系统的路径生成。通过添加随机误差、
%   累积误差和转弯误差，实现对实际惯导系统特性的仿真。
%
% 作者信息：
%   作者：Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.5
%   创建日期：240409
%   最后修改：240410
%
% 版本历史：
%   v1.0 (240409) - 首次发布
%       + 实现基础的惯导误差模拟
%       + 支持直线段和转弯段误差区分
%   v1.5 (240410) - 功能增强
%       + 添加累积误差随直线段长度的动态调整
%       + 优化转弯段误差生成算法
%       + 增加无误差起始段设置
%       + 完善艏向角更新逻辑
%
% 输入参数：
%   gps_path                - [Nx3 double] GPS路径矩阵 [x, y, heading]
%                            必选参数，heading单位为度
%   line_error_std_x        - [double] 直线段X方向随机误差标准差
%                            必选参数，建议范围0.02-0.05
%   line_error_std_y        - [double] 直线段Y方向随机误差标准差
%                            必选参数，建议范围0.03-0.08
%   turn_error_std_x        - [double] 转弯段X方向随机误差标准差
%                            必选参数，建议范围0.0003-0.001
%   turn_error_std_y        - [double] 转弯段Y方向随机误差标准差
%                            必选参数，建议范围0.01-0.03
%   cumulative_error_factor_x - [double] X方向累积误差因子
%                              必选参数，建议范围0.015-0.025
%   cumulative_error_factor_y - [double] Y方向累积误差因子
%                              必选参数，建议范围0.002-0.005
%   turn_error_factor_x     - [double] 转弯段X方向误差系数
%                            必选参数，建议范围0.005-0.02
%   turn_error_factor_y     - [double] 转弯段Y方向误差系数
%                            必选参数，建议范围0.005-0.02
%   no_error_fraction       - [double] 起始无误差段比例
%                            必选参数，建议范围0.02-0.05
%   window_size            - [int] 艏向角平滑窗口大小
%                           必选参数，建议范围30-50
%
% 输出参数：
%   simulated_path         - [Nx3 double] 仿真后的惯导路径 [x', y', heading']
%   simulated_error        - [Nx3 double] 仿真产生的误差 [err_x, err_y, err_heading]
%
% 注意事项：
%   1. GPS路径需预先平滑处理以获得更好的仿真效果
%   2. 参数设置会显著影响仿真结果的真实性
%   3. 建议根据实际惯导系统特性调整参数
%   4. 转弯检测阈值(1.2度)可根据需要调整
%
% 调用示例：
%   % 示例1：典型水下航行场景
%   params.error.line_std = [0.03, 0.05];          % 直线段误差标准差
%   params.error.turn_std = [0.0005, 0.02];        % 转弯段误差标准差
%   params.error.cumulative = [0.019, 0.0025];     % 累积误差因子
%   params.error.turn_factor = [0.01, 0.01];       % 转弯误差系数
%   params.error.no_error_fraction = 0.03;         % 无误差路径比例
%   params.error.window_size = 40;                 % 滑动窗口大小
%   
%   [ins_path, ins_error] = generateSimulatedInsPath(gps_path, ...
%       params.error.line_std(1), params.error.line_std(2), ...
%       params.error.turn_std(1), params.error.turn_std(2), ...
%       params.error.cumulative(1), params.error.cumulative(2), ...
%       params.error.turn_factor(1), params.error.turn_factor(2), ...
%       params.error.no_error_fraction, ...
%       params.error.window_size);
%
%   % 示例2：高精度惯导场景（误差较小）
%   [ins_path, ins_error] = generateSimulatedInsPath(gps_path, ...
%       0.01, 0.01, 0.0001, 0.005, ...
%       0.005, 0.001, 0.005, 0.005, ...
%       0.05, 30);
%
% 依赖工具箱：
%   - Statistics and Machine Learning Toolbox (normrnd函数)
%
% 参见函数：
%   normrnd, diff, atan2, rad2deg

function [simulated_path, simulated_error] = generateSimulatedInsPath(gps_path, ...
                                                                      line_error_std_x, line_error_std_y, ...
                                                                      turn_error_std_x, turn_error_std_y, ...
                                                                      cumulative_error_factor_x, cumulative_error_factor_y, ...
                                                                      turn_error_factor_x, turn_error_factor_y, ...
                                                                      no_error_fraction, window_size)

    % 提取路径信息
    heading = gps_path(:, 3); % 提取艏向角

    % 滑动窗口计算艏向角变化
    num_points = size(gps_path, 1); % 路径点数量
    smoothed_heading_diff = zeros(num_points, 1); % 初始化平滑后的艏向角变化率

    for i = 1:num_points
        % 滑动窗口范围
        start_idx = max(1, i - floor(window_size / 2)); % 窗口起点
        end_idx = min(num_points, i + floor(window_size / 2)); % 窗口终点
        smoothed_heading_diff(i) = mean(abs(diff(heading(start_idx:end_idx)))); % 计算平滑的艏向角变化率
    end

    % 转弯检测阈值
    threshold = 1.2; % 艏向角变化率阈值，定义是否为转弯段

    % 判断路径类型：1-直线段，2-转弯段
    path_type = ones(num_points, 1); % 初始化路径类型，默认所有点为直线段
    path_type(smoothed_heading_diff > threshold) = 2; % 艏向角变化率超过阈值的点标记为转弯段

    % 初始化误差数据
    simulated_path = gps_path; % 仿真路径初始化
    simulated_error = zeros(num_points, 3); % 误差矩阵初始化，Nx3，包含 [x_error, y_error, heading_error]

    % 累积误差和转弯计数器
    cumulative_error = [0, 0]; % 初始化累积误差 [累积误差_x, 累积误差_y]
    turn_count = 0; % 转弯计数器，用于模拟转弯段误差的交替方向

    % 前百分之几路径不增加误差的索引阈值
    no_error_length = round(no_error_fraction * num_points);

    % 计算每段直线长度并找出最长直线段
    max_straight_length = 0; % 初始化最长直线段长度
    current_length = 0; % 当前直线段长度
    for i = 2:num_points
        if path_type(i) == 1 % 当前点为直线段
            % 计算当前点与前一个点的距离
            segment_length = sqrt((gps_path(i, 1) - gps_path(i - 1, 1))^2 + (gps_path(i, 2) - gps_path(i - 1, 2))^2);
            current_length = current_length + segment_length; % 累加直线段长度
        else
            % 如果到达转弯段，更新最长直线段长度
            max_straight_length = max(max_straight_length, current_length);
            current_length = 0; % 重置当前直线段长度
        end
    end
    % 检查最后一段是否为最长直线段
    max_straight_length = max(max_straight_length, current_length);

    % 生成误差
    current_straight_length = 0; % 初始化当前直线段累积长度
    for i = 1:num_points
        if i <= no_error_length
            % 前百分之几路径不增加误差
            error_x = 0;
            error_y = 0;
            error_heading = 0; % 艏向角误差初始化为0
        elseif path_type(i) == 1 % 当前点为直线段
            % 计算当前点与前一个点的距离
            if i > 1
                segment_length = sqrt((gps_path(i, 1) - gps_path(i - 1, 1))^2 + (gps_path(i, 2) - gps_path(i - 1, 2))^2);
                current_straight_length = current_straight_length + segment_length; % 累加直线段长度
            end

            % 动态调整累积误差因子比例
            length_ratio = current_straight_length / max_straight_length; % 当前直线段长度占最长直线段的比例
            adjusted_cumulative_factor_x = cumulative_error_factor_x * length_ratio;
            adjusted_cumulative_factor_y = cumulative_error_factor_y * length_ratio;

            % 累计累积误差
            cumulative_error(1) = cumulative_error(1) + adjusted_cumulative_factor_x * segment_length;
            cumulative_error(2) = cumulative_error(2) + adjusted_cumulative_factor_y * segment_length;

            % 随机游走误差 (直线部分)
            error_x = normrnd(0, line_error_std_x);
            error_y = normrnd(0, line_error_std_y);
            error_heading = normrnd(0, line_error_std_x / 10); % 艏向角随机游走误差
        else % 当前点为转弯段
            % 重置当前直线段累积长度
            current_straight_length = 0;

            % 转弯误差突变
            turn_angle = smoothed_heading_diff(i); % 当前点滑动窗口艏向角变化量
            error_x = turn_error_factor_x * turn_angle + normrnd(0, turn_error_std_x);
            error_y = turn_error_factor_y * turn_angle * (-1)^turn_count + normrnd(0, turn_error_std_y);
            error_heading = turn_error_factor_x * turn_angle + normrnd(0, turn_error_std_x / 10); % 艏向角突变误差
            turn_count = turn_count + 1; % 转弯方向交替

            % 增加转弯误差到累积误差
            cumulative_error(1) = cumulative_error(1) + error_x;
            cumulative_error(2) = cumulative_error(2) + error_y;
        end

        % 总误差 = 累积误差 + 随机误差
        simulated_error(i, :) = [cumulative_error(1) + error_x, cumulative_error(2) + error_y, error_heading];
    end

    % 添加误差到路径
    simulated_path(:, 1:2) = gps_path(:, 1:2) + simulated_error(:, 1:2); % 更新路径的 x 和 y 坐标

    % 更新误差后的路径艏向角
    num_points = size(simulated_path, 1);
    updated_heading = zeros(num_points, 1); % 初始化更新后的艏向角
    for i = 1:num_points - 1
        dx = simulated_path(i + 1, 1) - simulated_path(i, 1);
        dy = simulated_path(i + 1, 2) - simulated_path(i, 2);
        updated_heading(i) = rad2deg(atan2(dy, dx)); % 计算艏向角并转换为角度制
    end
    updated_heading(end) = updated_heading(end - 1); % 最后一段艏向角延续前一段
    simulated_path(:, 3) = updated_heading; % 更新艏向角
end
