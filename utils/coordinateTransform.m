% coordinateTransform - 子地图坐标转换主程序
% 日期：241228
% 作者：李琦，Chihong（游子昂）
% 版本：v1.0
% 功能：
%   1. 加载子地图数据
%   2. 选取中间帧作为关键帧
%   3. 将所有测深点转换到关键帧坐标系
%   4. 生成转换后的子地图文件
%   5. 保存关键帧信息
% 输出文件：
%   - sub_map_*.txt：转换后的子地图数据
%   - KEY_FRAME.mat：关键帧信息

function KEY_FRAME = coordinateTransform(sub_maps, folder_save)
    % 获取子地图总数
    num_submaps = length(sub_maps);
    % 预分配关键帧矩阵 - 每行存储[索引,x,y,z,方向角]
    KEY_FRAME = zeros(num_submaps, 5);

    % 检查并创建并行池
    if isempty(gcp('nocreate'))
        parpool('Processes');
    end

    % 并行处理每个子地图
    parfor i = 1:num_submaps
        % 获取当前子地图
        A = sub_maps{i};
        % 计算中间帧位置
        mid_index = ceil(size(A, 1) / 2);
        % 提取关键帧数据
        key_frame = A(mid_index, :);
        
        % 提取关键帧信息
        key_frame_index = key_frame(1);                % 关键帧索引
        key_frame_position = key_frame([2, 3, 5]);     % 关键帧位置[x,y,z]
        key_frame_direction = key_frame(4);            % 关键帧方向角
        
        % 预分配存储转换点的数组
        total_points = sum((size(A, 2) - 5) / 3 * ones(size(A, 1), 1));
        transformed_points = zeros(total_points, 3);
        point_counter = 1;
        
        % 处理所有帧的测深点
        for j = 1:size(A, 1)
            frame = A(j, :);
            % 计算当前帧的测深点数量
            num_points = (size(frame, 2) - 5) / 3;
            % 重塑测深点数据为n*3矩阵
            points = reshape(frame(6:end), 3, num_points)';
            
            % 向量化处理测深点坐标转换
            for k = 1:size(points, 1)
                transformed_point = World2Body([key_frame_position, key_frame_direction], points(k, :));
                transformed_points(point_counter, :) = transformed_point;
                point_counter = point_counter + 1;
            end
        end
        
        % 清理数据：移除多余空间和NaN值
        transformed_points = transformed_points(1:point_counter-1, :);
        transformed_points = transformed_points(~any(isnan(transformed_points), 2), :);
        
        % 生成输出文件
        filename = fullfile(folder_save, sprintf('sub_map_%d.txt', i));
        write_submap_file(filename, key_frame_index, key_frame_position, key_frame_direction, transformed_points);
        
        % 存储关键帧信息
        KEY_FRAME(i,:) = [key_frame_index, key_frame_position, key_frame_direction];
    end
end

% 子函数：写入子地图文件
function write_submap_file(filename, key_frame_index, key_frame_position, key_frame_direction, transformed_points)
    fid = fopen(filename, 'w');
    % 写入关键帧信息
    fprintf(fid, '%d\n', key_frame_index);
    fprintf(fid, '%.6f %.6f %.6f\n', key_frame_position);
    fprintf(fid, '%.6f\n', key_frame_direction);
    % 写入转换后的测深点
    fprintf(fid, '%.6f %.6f %.6f\n', transformed_points');
    fclose(fid);
end