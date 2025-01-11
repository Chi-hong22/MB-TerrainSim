%% coordinateTransform - 子地图坐标转换处理工具
%
% 功能描述：
%   加载子地图数据，选取中间帧作为关键帧，将所有测深点转换到关键帧坐标系
%
% 作者信息：
%   作者：李琦，Chihong（游子昂）
%   邮箱：you.ziang@hrbeu.edu.cn
%   单位：哈尔滨工程大学
%
% 版本信息：
%   当前版本：v1.0
%   创建日期：241228
%   最后修改：241228
%
% 版本历史：
%   v1.0 (241228) - 首次发布
%       + 实现基础坐标转换功能
%       + 添加并行处理支持
%       + 实现关键帧提取
%
% 输入参数：
%   sub_maps    - 子地图数据元胞数组
%                 每个元素为一个矩阵，包含帧数据和点云信息
%   folder_save - 输出文件保存路径
%                 字符串，指定保存位置
%
% 输出参数：
%   KEY_FRAME   - 关键帧信息矩阵 [Nx5]
%                 每行：[索引,x,y,z,方向角]
%
% 注意事项：
%   1. 内存要求：与子地图数量和点云密度相关
%   2. 并行处理：自动调用并行池
%   3. 坐标系：采用右手坐标系
%   4. 文件操作：确保有写入权限
%
% 调用示例：
%   % 基础调用
%   KEY_FRAME = coordinateTransform(sub_maps, './output/');
%
% 依赖工具箱：
%   - Parallel Computing Toolbox
%
% 参见函数：
%   parpool, World2Body

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
        try
            % 获取当前子地图
            A = sub_maps{i};
            validateSubMapFormat(A);  % 添加数据格式验证
            
            % 计算中间帧位置
            mid_index = ceil(size(A, 1) / 2);
            % 提取关键帧数据
            key_frame = A(mid_index, :);
            
            % 提取关键帧信息
            key_frame_index = key_frame(1);                % 关键帧索引
            key_frame_position = key_frame([2, 3, 5]);     % 关键帧位置[x,y,z]
            key_frame_direction = key_frame(4);            % 关键帧方向角
            
            % 优化点云处理：一次性提取所有点
            points_all = [];
            frame_info = [];  % 存储每个点对应的帧信息
            
            for j = 1:size(A, 1)
                frame = A(j, :);
                num_points = (size(frame, 2) - 5) / 3;
                points = reshape(frame(6:end), 3, num_points)';
                points_all = [points_all; points];
                frame_info = [frame_info; repmat([frame(1:5)], num_points, 1)];
            end
            
            % 批量坐标转换
            transformed_points = World2Body([key_frame_position, key_frame_direction], points_all);
            
            % 验证转换结果
            validateResults(transformed_points, key_frame_position);
            
            % 清理数据：移除多余空间和NaN值
            transformed_points = transformed_points(~any(isnan(transformed_points), 2), :);
            
            % 生成输出文件
            filename = fullfile(folder_save, sprintf('sub_map_%d.txt', i));
            write_submap_file(filename, key_frame_index, key_frame_position, key_frame_direction, transformed_points);
            
            % 存储关键帧信息
            KEY_FRAME(i,:) = [key_frame_index, key_frame_position, key_frame_direction];
        catch ME
            warning('处理子地图 %d 时出错: %s', i, ME.message);
            continue;
        end
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

function validateSubMapFormat(submap)
    if size(submap,2) < 6
        error('子地图数据格式错误：列数不足');
    end
    if mod(size(submap,2)-5, 3) ~= 0
        error('子地图数据格式错误：点云数据不完整');
    end
end

function validateResults(points, ref_pos)
    % 设置合理的阈值
    max_dist = 2000; % meters
    min_dist = 0.1;  % meters
    
    distances = vecnorm(points - ref_pos, 2, 2);
    
    if any(distances > max_dist)
        warning('检测到异常远点: %d 个点超出阈值', sum(distances > max_dist));
    end
    
    if any(distances < min_dist)
        warning('检测到异常近点: %d 个点小于阈值', sum(distances < min_dist));
    end
end