function [simulated_path, simulated_error] = generateSimulatedInsPath(gps_path, ...
                                                                      line_error_std_x, line_error_std_y, ...
                                                                      turn_error_std_x, turn_error_std_y, ...
                                                                      cumulative_error_factor_x, cumulative_error_factor_y, ...
                                                                      turn_error_factor_x, turn_error_factor_y, ...
                                                                      no_error_fraction, window_size)
    % 函数说明：
    % generateSimulatedInsPath 模拟惯性导航系统（INS）的路径，基于输入的GPS路径数据添加位置误差并生成仿真的INS路径。
    % 日期：241226
    % 作者：Chihong（游子昂）
    % 版本：1.5
    %
    % 输入参数：
    % gps_path                - GPS路径，Nx3矩阵，每行包含 [x, y, heading]
    % line_error_std_x        - 直线段 X 方向误差的标准差
    % line_error_std_y        - 直线段 Y 方向误差的标准差
    % turn_error_std_x        - 转弯段 X 方向误差的标准差
    % turn_error_std_y        - 转弯段 Y 方向误差的标准差
    % cumulative_error_factor_x - 累积误差的 X 方向因子
    % cumulative_error_factor_y - 累积误差的 Y 方向因子
    % turn_error_factor_x     - 转弯段 X 方向误差的系数
    % turn_error_factor_y     - 转弯段 Y 方向误差的系数
    % no_error_fraction       - 前百分之几的路径不产生误差 (范围 0-1)
    % window_size             - 滑动窗口大小，用于计算路径的平滑艏向角变化
    %
    % 输出参数：
    % simulated_path          - 仿真后的INS路径，Nx3矩阵，每行包含 [x', y', heading']
    % simulated_error         - 仿真路径中的误差，Nx3矩阵，每行包含 [误差_x, 误差_y, 误差_heading]，单位为角度制

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
