% submap2PCD - 子地图规范化处理脚本
% 日期：241228
% 作者：Chihong（游子昂）
% 版本：v1.0
% 功能：
%   1. 读取子地图文本文件
%   2. 提取关键帧信息和测深点数据
%   3. 将数据转换为标准PCD点云格式
%   4. 生成带有位姿信息的PCD文件
% 输入：子地图文本文件（sub_map_*.txt）
% 输出：标准PCD格式文件（submap_*_frame.pdc）
% 
% 读取指定目录下的所有sub_map_i.txt，将他们转化为bathymetric_slam需要的数据格式。
% 其中速度方向转化为了四元数的形式。存为submap_i_frame.pdc

function submap2PCD(inputDir_TXT, outputDir_PCD)
    % 获取文件列表
    files = dir(fullfile(inputDir_TXT, 'sub_map_*.txt'));
    assert(~isempty(files), '未找到sub_map_*.txt文件');
    
    % 文件排序
    [~, order] = sort(arrayfun(@(x) sscanf(x.name, 'sub_map_%d.txt'), files));
    files = files(order);
    
    % 创建并行池
    if isempty(gcp('nocreate'))
        parpool('Processes');
    end
    
    % 并行处理文件
    parfor fileIdx = 1:length(files)
        file = files(fileIdx);
        process_single_file(inputDir_TXT, outputDir_PCD, file, fileIdx);
    end
end

function process_single_file(inputDir_TXT, outputDir_PCD, file, fileIdx)
    % 读取数据
    filePath = fullfile(inputDir_TXT, file.name);
    data = dlmread(filePath);
    
    % 提取数据
    keyframe_index = data(1, :);
    keyframe_position = data(2, :);
    phi = data(3, 1);
    points = data(4:end, :);
    num_points = size(points, 1);
    
    % 计算四元数
    q = Phi2Quaternion(phi);
    
    % 生成PCD头部
    pcdHeader = generatePCDHeader(keyframe_position, q, num_points);
    
    % 写入文件
    outputFilePath = fullfile(outputDir_PCD, sprintf('submap_%d_frame.pdc', fileIdx));
    writeDataToPCD(outputFilePath, pcdHeader, points);
end

function header = generatePCDHeader(position, q, num_points)
    header = sprintf(['# .PCD v0.7 - Point Cloud Data file format\n' ...
                     'VERSION 0.7\n' ...
                     'FIELDS x y z\n' ...
                     'SIZE 4 4 4\n' ...
                     'TYPE F F F\n' ...
                     'COUNT 1 1 1\n' ...
                     'WIDTH %d\n' ...
                     'HEIGHT 1\n' ...
                     'VIEWPOINT %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n' ...
                     'POINTS %d\n' ...
                     'DATA ascii\n'], ...
                     num_points, position(1), position(2), position(3), ...
                     q(1), q(2), q(3), q(4), num_points);
end

function writeDataToPCD(filepath, header, points)
    fid = fopen(filepath, 'w');
    fprintf(fid, '%s', header);
    fclose(fid);
    dlmwrite(filepath, points, '-append', 'delimiter', ' ', 'precision', 7);
end