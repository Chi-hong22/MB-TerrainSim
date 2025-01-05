% deleteNan - 清理PCD文件中的无效数据
% 功能：
%   1. 读取PCD格式文件
%   2. 删除包含NaN值的点
%   3. 更新点云文件的头部信息
%   4. 保存清理后的PCD文件
% 参数：
%   input_folder - 输入文件夹路径
%   output_folder - 输出文件夹路径

function deleteNan(input_folder, output_folder)
    % 列出输入文件夹中所有的 .pdc 文件
    files = dir(fullfile(input_folder, 'submap_*_frame.pdc'));
    
    % 提取文件名中的数字部分并按升序排序
    file_indices = arrayfun(@(f) sscanf(f.name, 'submap_%d_frame.pdc'), files);
    [~, sorted_indices] = sort(file_indices);
    files = files(sorted_indices);

    % 遍历排序后的 .pdc 文件
    for i = 1:length(files)
        % 获取当前文件路径
        file_path = fullfile(files(i).folder, files(i).name);
        
        % 读取文件内容
        file_content = fileread(file_path);
        file_lines = strsplit(file_content, '\n'); % 按行分割内容
        
        % 找到 DATA ascii 行及其后的点数据
        data_start_idx = find(contains(file_lines, 'DATA ascii'), 1, 'first');
        if isempty(data_start_idx)
            warning('File %s is missing "DATA ascii" line. Skipping...', files(i).name);
            continue;
        end
        
        % 提取点数据并转换为矩阵
        point_lines = file_lines(data_start_idx + 1:end); % 数据部分
        points = cellfun(@(line) sscanf(line, '%f %f %f')', point_lines, 'UniformOutput', false);
        points = vertcat(points{:}); % 转换为 n*3 矩阵
        
        % 删除包含 NaN 的点
        valid_points = points(~any(isnan(points), 2), :);
        
        % 更新文件内容
        file_lines(data_start_idx + 1:end) = arrayfun(@(row) sprintf('%.6f %.6f %.6f', ...
            valid_points(row, 1), valid_points(row, 2), valid_points(row, 3)), ...
            1:size(valid_points, 1), 'UniformOutput', false);
        
        % 更新 WIDTH 和 POINTS 行
        num_points = size(valid_points, 1);
        file_lines{7} = regexprep(file_lines{7}, '(WIDTH )\d+', sprintf('WIDTH %d', num_points));
        file_lines{10} = regexprep(file_lines{10}, '(POINTS )\d+', sprintf('POINTS %d', num_points));
        
        % 组合新的文件内容
        updated_content = strjoin(file_lines, '\n');
        
        % 保存到输出文件夹下的同名文件
        output_path = fullfile(output_folder, files(i).name);
        fid = fopen(output_path, 'w');
        fprintf(fid, '%s', updated_content);
        fclose(fid);
    end
end
