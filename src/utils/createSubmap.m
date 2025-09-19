% createSubmap - 多波束数据分段与子地图创建函数
% 日期：250104
% 作者：Chihong（游子昂），李琦
% 版本：v1.4 
% 更新内容：
%   1. 引入双通道处理机制：同时处理理想数据和含噪声数据
%   2. 优化子地图划分算法：基于理想数据进行分段
%   3. 新增可视化功能：支持理想与噪声数据对照显示
% 历史版本: v1.3 优化分割逻辑——仅处理直线段数据
%
% 输入参数:
%   recoder - 原始多波束数据记录矩阵
%       第1列：帧数
%       第2-3列：AUV的xy坐标
%       第4列：艏向角（角度制）
%       第5列：标识符
%       第6列及之后：多波束点云数据(256点)
%   params - 参数结构体
%       .window_size - 居中滑动窗口大小(默认:40帧)
%       .heading_threshold - 平均艏向角变化率阈值(默认:0.9度/采样点)
%       .frames_per_submap - 每个子地图的帧数(默认:100帧)
%
% 输出参数:
%   sub_maps - 包含所有子地图数据的cell数组
%   segment_info - 包含分段信息的结构体
%       .type - 段类型(1:直线段, 2:转弯段)
%       .start_idx - 起始索引
%       .end_idx - 结束索引
%
% 分割逻辑：
%   1. 长直线段处理 (>=100帧):
%       - 按100帧划分为完整子地图
%       - 最后剩余部分保留为一个独立子地图
%   2. 短直线段处理 (<100帧):
%       - 当两侧为转弯段且直线段长度≥75帧时:
%           * 从相邻转弯段补充数据至100帧
%           * 优先从上一转弯段补充，避免与已有子地图重叠
%           * 剩余帧数从下一转弯段起始处补充
%       - 当上一段为直线段时:
%           * 直接并入上一个子地图
%       - 当为最后一个直线段时:
%           * 并入上一个子地图
%   3. 转弯段处理:
%       - 仅作为直线段补充数据来源
%       - 不单独构成子地图
%   4. 重叠处理:
%       - 确保新子地图与已有子地图不重叠

function [sub_maps, sub_maps_with_error, segment_info] = createSubmap(recoder, recoder_with_ins_error, params)

    % 1. 参数初始化
    if nargin < 3
        params = struct();
    end
    if ~isfield(params, 'window_size')
        params.window_size = 40;
    end
    if ~isfield(params, 'heading_threshold')
        params.heading_threshold =  0.9; ...1.22
    end
    if ~isfield(params, 'frames_per_submap')
        params.frames_per_submap = 100;
    end
    
    % 2. 使用滑动窗口检测直线段和转弯段（使用无误差数据进行检测）
    segment_info = detectSegments(recoder(:,4), params.window_size, params.heading_threshold);
    
    % 3. 创建两组子地图
    [sub_maps, sub_maps_with_error] = createSubmapsFromSegments(recoder, recoder_with_ins_error, segment_info, params.frames_per_submap, 75);
    
    % 4. 可视化结果
    visualizeSubmapSeg(recoder, recoder_with_ins_error, segment_info, sub_maps, sub_maps_with_error);

end

%% 辅助函数

function segment_info = detectSegments(heading, window_size, threshold)
    % detectSegments - 使用滑动窗口检测直线段和转弯段
    %
    % 输入:
    % heading - 艏向角数据向量
    % window_size - 滑动窗口大小
    % threshold - 艏向角变化阈值
    %
    % 输出:
    % segment_info - 包含每个段落信息的结构体数组
    %   .type - 段落类型(1:直线段, 2:转弯段)
    %   .start_idx - 起始索引
    %   .end_idx - 结束索引
    
    n = length(heading);
    smoothed_heading_diff = zeros(n, 1);
    
    % 计算居中滑动窗口内的平均艏向角变化率
    for i = 1:n
        % 计算窗口范围
        start_idx = max(1, i - floor(window_size/2));
        end_idx = min(n, i + floor(window_size/2));
        
        % 计算窗口内的平均艏向角变化率
        window_heading = heading(start_idx:end_idx);
        smoothed_heading_diff(i) = mean(abs(diff(window_heading)));
    end
    
    % 判断路径类型
    is_turning = smoothed_heading_diff > threshold;
    
    % 合并连续的段落
    segment_info = struct('type', {}, 'start_idx', {}, 'end_idx', {});
    current_type = ~is_turning(1);
    start_idx = 1;
    seg_idx = 1;
    
    for i = 2:n
        if is_turning(i) ~= is_turning(i-1) || i == n
            % 当段落类型改变或到达终点时，保存当前段落
            if current_type
                segment_info(seg_idx).type = 1;  % 直线段
            else
                segment_info(seg_idx).type = 2;  % 转弯段
            end
            segment_info(seg_idx).start_idx = start_idx;
            segment_info(seg_idx).end_idx = i-1;
            
            % 准备下一个段落
            start_idx = i;
            current_type = ~current_type;
            seg_idx = seg_idx + 1;
        end
    end
    
    % 处理最后一个段落(如果没有在循环中处理)
    if start_idx <= n
        if current_type
            segment_info(seg_idx).type = 1;  % 直线段
        else
            segment_info(seg_idx).type = 2;  % 转弯段
        end
        segment_info(seg_idx).start_idx = start_idx;
        segment_info(seg_idx).end_idx = n;
    end
end


function [sub_maps, sub_maps_with_error] = createSubmapsFromSegments(recoder, recoder_with_ins_error, segment_info, frames_per_submap, min_straight_frames)
    % createSubmapsFromSegments - 根据分段信息创建子地图
    %
    % 输入参数:
    % recoder - 原始数据
    % recoder_with_ins_error - 带误差的数据
    % segment_info - 分段信息
    % frames_per_submap - 每个子地图的目标帧数（默认100帧）
    % min_straight_frames - 最小直线段帧数阈值（默认75帧）
    
    if nargin < 4
        min_straight_frames = 75;
    end
    
    sub_maps = {};
    sub_maps_with_error = {};
    map_idx = 1;
    last_used_frame = 0;  % 记录上一个子地图使用的最后一帧位置
    
    for i = 1:length(segment_info)
        if segment_info(i).type == 1  % 直线段
            current_segment = recoder(segment_info(i).start_idx:segment_info(i).end_idx, :);
            current_segment_error = recoder_with_ins_error(segment_info(i).start_idx:segment_info(i).end_idx, :);
            segment_length = size(current_segment, 1);
            
            if segment_length >= frames_per_submap
                % 大于等于100帧的直线段，直接分割成子地图
                num_submaps = floor(segment_length / frames_per_submap);
                for j = 1:num_submaps-1  % 注意这里改为num_submaps-1
                    start_idx = (j-1) * frames_per_submap + 1;
                    end_idx = j * frames_per_submap;
                    sub_maps{map_idx} = current_segment(start_idx:end_idx, :);
                    sub_maps_with_error{map_idx} = current_segment_error(start_idx:end_idx, :);
                    last_used_frame = segment_info(i).start_idx + end_idx - 1;
                    map_idx = map_idx + 1;
                end
                
                % 将最后一个完整子地图和剩余帧合并处理
                remaining_frames = segment_length - (num_submaps-1) * frames_per_submap;
                start_idx = (num_submaps-1) * frames_per_submap + 1;
                sub_maps{map_idx} = current_segment(start_idx:end, :);
                sub_maps_with_error{map_idx} = current_segment_error(start_idx:end, :);
                last_used_frame = segment_info(i).end_idx;
                map_idx = map_idx + 1;
            else
                % 处理小于frames_per_submap的直线段
                if i > 1 && i < length(segment_info)  % 不是第一段也不是最后一段
                    if segment_info(i-1).type == 2 && segment_info(i+1).type == 2
                        % 情况1：两侧都是转弯段
                        if segment_length >= min_straight_frames
                            frames_needed = frames_per_submap - segment_length;
                            frames_per_side = ceil(frames_needed / 2);  % 每侧需要补充的帧数
                            
                            % 获取前一转弯段数据（考虑避免与上一子地图重叠）
                            if last_used_frame >= segment_info(i-1).start_idx && last_used_frame <= segment_info(i-1).end_idx
                                % 存在重叠，从last_used_frame后一帧开始取
                                valid_start = last_used_frame + 1;
                                prev_available_frames = recoder(valid_start:segment_info(i-1).end_idx, :);
                                prev_available_frames_error = recoder_with_ins_error(valid_start:segment_info(i-1).end_idx, :);
                                prev_frames = prev_available_frames(end-min(frames_per_side,size(prev_available_frames,1))+1:end, :);
                                prev_frames_error = prev_available_frames_error(end-min(frames_per_side,size(prev_available_frames_error,1))+1:end, :);
                            else
                                % 不存在重叠，从转弯段末尾取帧
                                prev_turning = recoder(segment_info(i-1).start_idx:segment_info(i-1).end_idx, :);
                                prev_turning_error = recoder_with_ins_error(segment_info(i-1).start_idx:segment_info(i-1).end_idx, :);
                                prev_frames = prev_turning(end-min(frames_per_side,size(prev_turning,1))+1:end, :);
                                prev_frames_error = prev_turning_error(end-min(frames_per_side,size(prev_turning_error,1))+1:end, :);
                            end
                            
                            % 获取后一转弯段数据（从开头取帧）
                            next_turning = recoder(segment_info(i+1).start_idx:segment_info(i+1).end_idx, :);
                            next_turning_error = recoder_with_ins_error(segment_info(i+1).start_idx:segment_info(i+1).end_idx, :);
                            next_frames_needed = frames_per_submap - segment_length - size(prev_frames, 1);
                            next_frames = next_turning(1:min(next_frames_needed,size(next_turning,1)), :);
                            next_frames_error = next_turning_error(1:min(next_frames_needed,size(next_turning_error,1)), :);
                            
                            % 组合子地图
                            sub_maps{map_idx} = [prev_frames; current_segment; next_frames];
                            sub_maps_with_error{map_idx} = [prev_frames_error; current_segment_error; next_frames_error];
                            last_used_frame = segment_info(i+1).start_idx + size(next_frames, 1) - 1;
                            map_idx = map_idx + 1;
                        end
                        % 如果小于阈值，直接跳过，不创建子地图
                    elseif segment_info(i-1).type == 1
                        % 情况2：上一个为直线段，并入上一个子地图
                        if map_idx > 1
                            sub_maps{map_idx-1} = [sub_maps{map_idx-1}; current_segment];
                            sub_maps_with_error{map_idx-1} = [sub_maps_with_error{map_idx-1}; current_segment_error];
                            last_used_frame = segment_info(i).end_idx;
                        end
                    end
                elseif i == length(segment_info)
                    % 情况3：最后一个子地图不满100帧，并入上一个子地图
                    if map_idx > 1
                        sub_maps{map_idx-1} = [sub_maps{map_idx-1}; current_segment];
                        sub_maps_with_error{map_idx-1} = [sub_maps_with_error{map_idx-1}; current_segment_error];
                        last_used_frame = segment_info(i).end_idx;
                    end
                end
            end
        end
    end
end

function visualizeSubmapSeg(recoder, recoder_with_ins_error, segment_info, sub_maps, sub_maps_with_error)
    % 1. 显示直线段/转弯段划分情况
    figure('Name', '直线/转弯分割可视化', 'Position', [100 100 1000 400]);
    
    % 创建左子图
    subplot('Position', [0.1 0.1 0.35 0.8]);
    hold on;
    title('无误差数据分段结果');
    % 绘制直线段和转弯段
    for i = 1:length(segment_info)
        segment_data = recoder(segment_info(i).start_idx:segment_info(i).end_idx, :);
        if segment_info(i).type == 1 % 直线段
            plot(segment_data(:,2), segment_data(:,3), 'b-', 'LineWidth', 1.5);
        else % 转弯段
            plot(segment_data(:,2), segment_data(:,3), 'r--', 'LineWidth', 1.5);
        end
    end
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    axis equal;
    legend('直线段', '转弯段');
    hold off;
    
    % 创建右子图
    subplot('Position', [0.6 0.1 0.35 0.8]);
    hold on;
    title('带误差数据分段结果');
    % 绘制带误差数据的直线段和转弯段
    for i = 1:length(segment_info)
        segment_data = recoder_with_ins_error(segment_info(i).start_idx:segment_info(i).end_idx, :);
        if segment_info(i).type == 1 % 直线段
            plot(segment_data(:,2), segment_data(:,3), 'b-', 'LineWidth', 1.5);
        else % 转弯段
            plot(segment_data(:,2), segment_data(:,3), 'r--', 'LineWidth', 1.5);
        end
    end
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    axis equal;
    legend('直线段', '转弯段');
    hold off;
    
    % 2. 显示子地图划分情况
    figure('Name', '子地图分割可视化', 'Position', [200 100 1000 400]);
    
    % 创建左子图
    subplot('Position', [0.1 0.1 0.35 0.8]);
    hold on;
    title('无误差数据子地图');
    colors = ['r', 'b', 'g']; % 交替颜色
    for i = 1:length(sub_maps)
        submap = sub_maps{i};
        color = colors(mod(i - 1, length(colors)) + 1);
        plot(submap(:,2), submap(:,3), [color, '.'], 'LineWidth', 1.5);
    end
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    axis equal;
    hold off;
    
    % 创建右子图
    subplot('Position', [0.6 0.1 0.35 0.8]);
    hold on;
    title('带误差数据子地图');
    for i = 1:length(sub_maps_with_error)
        submap = sub_maps_with_error{i};
        color = colors(mod(i - 1, length(colors)) + 1);
        plot(submap(:,2), submap(:,3), [color, '.'], 'LineWidth', 1.5);
    end
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    axis equal;
    hold off;
end


